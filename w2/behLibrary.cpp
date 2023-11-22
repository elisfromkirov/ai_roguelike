#include "aiLibrary.h"
#include "behaviourTree.h"
#include "ecsTypes.h"
#include "aiUtils.h"
#include "flecs/addons/cpp/entity.hpp"
#include "math.h"
#include "raylib.h"
#include "blackboard.h"
#include <limits>
#include <memory>

class CompoundNode : public BehNode
{
protected:
  std::vector<std::unique_ptr<BehNode>> nodes_;

public:
  CompoundNode(const std::vector<BehNode*>& nodes)
  {
    for (auto node : nodes)
    {
      nodes_.emplace_back(node);
    }
  }

  CompoundNode &pushNode(BehNode *node)
  {
    nodes_.emplace_back(node);
    return *this;
  }
};

class Sequence : public CompoundNode
{
public:
  Sequence(const std::vector<BehNode*>& nodes)
    : CompoundNode(nodes)
  {
  }

  BehResult update(flecs::world &world, flecs::entity entity, Blackboard &blackboard) override
  {
    for (const auto& node : nodes_)
    {
      auto result = node->update(world, entity, blackboard);
      if (result != BEH_SUCCESS)
      {
        return result;
      }
    }
    return BEH_SUCCESS;
  }
};

class Selector : public CompoundNode
{
public:
  Selector(const std::vector<BehNode*>& nodes)
    : CompoundNode(nodes)
  {
  }

  BehResult update(flecs::world &world, flecs::entity entity, Blackboard &blackboard) override
  {
    for (const auto& node : nodes_)
    {
      auto result = node->update(world, entity, blackboard);
      if (result != BEH_FAIL)
      {
        return result;
      }
    }
    return BEH_FAIL;
  }
};

class ParallelNode : public CompoundNode
{
public:
  ParallelNode(const std::vector<BehNode*>& nodes)
    : CompoundNode(nodes)
  {
  }

  BehResult update(flecs::world &world, flecs::entity entity, Blackboard &blackboard) override
  {
    for (const auto& node : nodes_)
    {
      auto result = node->update(world, entity, blackboard);
      if (result != BEH_RUNNING)
      {
        return result;
      }
    }
    return BEH_RUNNING;
  }
};

class NegatorNode : public BehNode
{
  std::unique_ptr<BehNode> node_;

public:
  NegatorNode(BehNode* node)
    : node_{node}
  {
  }

  BehResult update(flecs::world &world, flecs::entity entity, Blackboard &blackboard) override
  {
    auto result = node_->update(world, entity, blackboard);
    return result == BEH_FAIL ? BEH_SUCCESS : BEH_FAIL;
  }
};

struct MoveToEntity : public BehNode
{
  size_t entityBb = size_t(-1); // wraps to 0xff...
  MoveToEntity(flecs::entity entity, const char *bb_name)
  {
    entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
  }

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_RUNNING;
    entity.set([&](Action &a, const Position &pos)
    {
      flecs::entity targetEntity = bb.get<flecs::entity>(entityBb);
      if (!targetEntity.is_alive())
      {
        res = BEH_FAIL;
        return;
      }
      targetEntity.get([&](const Position &target_pos)
      {
        if (pos != target_pos)
        {
          a.action = move_towards(pos, target_pos);
          res = BEH_RUNNING;
        }
        else
          res = BEH_SUCCESS;
      });
    });
    return res;
  }
};

struct IsLowHp : public BehNode
{
  float threshold = 0.f;
  IsLowHp(float thres) : threshold(thres) {}

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &) override
  {
    BehResult res = BEH_SUCCESS;
    entity.get([&](const Hitpoints &hp)
    {
      res = hp.hitpoints < threshold ? BEH_SUCCESS : BEH_FAIL;
    });
    return res;
  }
};

struct FindEnemy : public BehNode
{
  size_t entityBb = size_t(-1);
  float distance = 0;
  FindEnemy(flecs::entity entity, float in_dist, const char *bb_name) : distance(in_dist)
  {
    entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
  }
  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_FAIL;
    static auto enemiesQuery = ecs.query<const Position, const Team>();
    entity.set([&](const Position &pos, const Team &t)
    {
      flecs::entity closestEnemy;
      float closestDist = FLT_MAX;
      Position closestPos;
      enemiesQuery.each([&](flecs::entity enemy, const Position &epos, const Team &et)
      {
        if (t.team == et.team)
          return;
        float curDist = dist(epos, pos);
        if (curDist < closestDist)
        {
          closestDist = curDist;
          closestPos = epos;
          closestEnemy = enemy;
        }
      });
      if (ecs.is_valid(closestEnemy) && closestDist <= distance)
      {
        bb.set<flecs::entity>(entityBb, closestEnemy);
        res = BEH_SUCCESS;
      }
    });
    return res;
  }
};

struct Flee : public BehNode
{
  size_t entityBb = size_t(-1);
  Flee(flecs::entity entity, const char *bb_name)
  {
    entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
  }

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_RUNNING;
    entity.set([&](Action &a, const Position &pos)
    {
      flecs::entity targetEntity = bb.get<flecs::entity>(entityBb);
      if (!targetEntity.is_alive())
      {
        res = BEH_FAIL;
        return;
      }
      targetEntity.get([&](const Position &target_pos)
      {
        a.action = inverse_move(move_towards(pos, target_pos));
      });
    });
    return res;
  }
};

struct Patrol : public BehNode
{
  size_t pposBb = size_t(-1);
  float patrolDist = 1.f;
  Patrol(flecs::entity entity, float patrol_dist, const char *bb_name)
    : patrolDist(patrol_dist)
  {
    pposBb = reg_entity_blackboard_var<Position>(entity, bb_name);
    entity.set([&](Blackboard &bb, const Position &pos)
    {
      bb.set<Position>(pposBb, pos);
    });
  }

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_RUNNING;
    entity.set([&](Action &a, const Position &pos)
    {
      Position patrolPos = bb.get<Position>(pposBb);
      if (dist(pos, patrolPos) > patrolDist)
        a.action = move_towards(pos, patrolPos);
      else
        a.action = GetRandomValue(EA_MOVE_START, EA_MOVE_END - 1); // do a random walk
    });
    return res;
  }
};

class FindNearestBuff final : public BehNode
{
  size_t found_buff_index_;

public:
  FindNearestBuff(flecs::entity entity, const char *found_buff_name)
    : found_buff_index_{reg_entity_blackboard_var<flecs::entity>(entity, found_buff_name)}
  {
  }

  BehResult update(flecs::world &world, flecs::entity entity, Blackboard &blackboard) override
  {
    flecs::entity nearest_buff{};
    auto minimal_distance = std::numeric_limits<float>::max();

    if (!entity.has<Position>())
    {
      return BEH_FAIL;
    }

    auto position = *entity.get<Position>();

    world.query<Position, BuffTag>().each(
      [&](flecs::entity buff, const Position& buff_position, [[maybe_unused]] BuffTag buff_tag) {
        auto distance = dist(position, buff_position);
        if (minimal_distance > distance)
        {
          nearest_buff = buff;
          minimal_distance = distance;
        }
      }
    );

    blackboard.set(found_buff_index_, nearest_buff);

    return BEH_SUCCESS;
  }
};

class FindNextWayPoint : public BehNode
{
  size_t found_way_point_index_;

public:
  FindNextWayPoint(flecs::entity entity, flecs::entity way_point, const char *found_way_point_name)
    : found_way_point_index_{reg_entity_blackboard_var<flecs::entity>(entity, found_way_point_name)}
  {
    entity.get_mut<Blackboard>()->set<flecs::entity>(found_way_point_index_, way_point);
  }

  BehResult update([[maybe_unused]] flecs::world &world, [[maybe_unused]] flecs::entity entity, Blackboard &blackboard) override
  {
    auto next_way_point = blackboard.get<flecs::entity>(found_way_point_index_).get<WayPoint>()->next_way_point;
    blackboard.set<flecs::entity>(found_way_point_index_, next_way_point);
    return BEH_SUCCESS;
  }
};

BehNode *sequence(const std::vector<BehNode*> &nodes)
{
  return new Sequence{nodes};
}

BehNode *selector(const std::vector<BehNode*> &nodes)
{
  return new Selector{nodes};
}

BehNode *parallel(const std::vector<BehNode*> &nodes)
{
  return new ParallelNode{nodes};
}

BehNode *negate(BehNode *node)
{
  return new NegatorNode{node};
}

BehNode *move_to_entity(flecs::entity entity, const char *bb_name)
{
  return new MoveToEntity(entity, bb_name);
}

BehNode *is_low_hp(float thres)
{
  return new IsLowHp(thres);
}

BehNode *find_enemy(flecs::entity entity, float dist, const char *bb_name)
{
  return new FindEnemy(entity, dist, bb_name);
}

BehNode *flee(flecs::entity entity, const char *bb_name)
{
  return new Flee(entity, bb_name);
}

BehNode *patrol(flecs::entity entity, float patrol_dist, const char *bb_name)
{
  return new Patrol(entity, patrol_dist, bb_name);
}

BehNode *find_nearest_buff(flecs::entity entity, const char *found_buff_name)
{
  return new FindNearestBuff{entity, found_buff_name};
}

BehNode *find_next_way_point(flecs::entity entity, flecs::entity way_point, const char *found_way_point_name)
{
  return new FindNextWayPoint{entity, way_point, found_way_point_name};
}