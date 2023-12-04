#include "aiLibrary.h"
#include "aiUtils.h"
#include "blackboard.h"
#include "ecsTypes.h"
#include "math.h"
#include "raylib.h"

#include <algorithm>
#include <limits>
#include <random>

struct CompoundNode : public BehNode
{
  std::vector<BehNode*> nodes;

  virtual ~CompoundNode()
  {
    for (BehNode *node : nodes)
      delete node;
    nodes.clear();
  }

  CompoundNode &pushNode(BehNode *node)
  {
    nodes.push_back(node);
    return *this;
  }
};

struct Sequence : public CompoundNode
{
  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    for (BehNode *node : nodes)
    {
      BehResult res = node->update(ecs, entity, bb);
      if (res != BEH_SUCCESS)
        return res;
    }
    return BEH_SUCCESS;
  }
};

struct Selector : public CompoundNode
{
  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    for (BehNode *node : nodes)
    {
      BehResult res = node->update(ecs, entity, bb);
      if (res != BEH_FAIL)
        return res;
    }
    return BEH_FAIL;
  }
};

class UtilitySelector : public BehNode
{
  struct NodeWithUtility
  {
    std::unique_ptr<BehNode> node;
    Utility utility;
  };

protected:
  std::vector<NodeWithUtility> nodes_;

public:
  explicit UtilitySelector(const std::vector<std::pair<BehNode*, Utility>>& nodes)
    : nodes_{}
  {
    nodes_.reserve(nodes.size());
    for (const auto& [node, utility] : nodes)
    {
      nodes_.push_back({std::unique_ptr<BehNode>{node}, utility});
    }
  }

  BehResult update(flecs::world &world, flecs::entity entity, Blackboard &blackboard) override
  {
    std::vector<std::pair<size_t, float>> indexed_score{};
    indexed_score.reserve(nodes_.size());

    for (size_t index = 0; index < nodes_.size(); ++index)
    {
      auto score = nodes_[index].utility(blackboard);
      indexed_score.push_back(std::make_pair(index, score));
    }

    std::sort(indexed_score.begin(), indexed_score.end(), [](auto &lhs, auto &rhs) { return lhs.second > rhs.second; } );

    for (const auto& [index, score] : indexed_score)
    {
      if (auto result = nodes_[index].node->update(world, entity, blackboard); result != BEH_FAIL)
      {
        return result;
      }
    }

    return BEH_FAIL;
  }
};

class WeightedRandomUtilitySelector : public UtilitySelector
{
  std::default_random_engine random_engine_;

public:
  WeightedRandomUtilitySelector(const std::vector<std::pair<BehNode*, Utility>>& nodes)
    : UtilitySelector{nodes}
    , random_engine_{}
  {
  }

  BehResult update(flecs::world &world, flecs::entity entity, Blackboard &blackboard) override
  {
    std::vector<float> scores{};
    scores.reserve(nodes_.size());

    auto total_score = 0.f;
    for (size_t i = 0; i < nodes_.size(); ++i)
    {
      auto score = nodes_[i].utility(blackboard);
      scores.push_back(score);
      total_score += score;
    }

    std::uniform_real_distribution<float> distribution{0.f, total_score};

    for (size_t i = 0; i < nodes_.size(); ++i)
    {
      auto random_score = distribution(random_engine_);

      size_t index = 0;
      while (random_score > scores[index] && index < nodes_.size())
      {
        random_score -= scores[index];
      }

      if (auto result = nodes_[index].node->update(world, entity, blackboard); result != BEH_FAIL)
      {
        return result;
      }
    }

    return BEH_FAIL;
  }
};

class InertialUtilitySelector : public UtilitySelector
{
  std::vector<float> nodes_inertia_;
  float inertia_;
  float inertia_decrease_rate_;
  size_t previously_selected_node_index_;

public:
  InertialUtilitySelector(const std::vector<std::pair<BehNode*, Utility>>& nodes,
                          float inertia = 30.f, float inertia_decrease_rate = 10.f)
    : UtilitySelector{nodes}
    , nodes_inertia_(nodes_.size(), 0.f)
    , inertia_{inertia}
    , inertia_decrease_rate_{inertia_decrease_rate}
    , previously_selected_node_index_{std::numeric_limits<size_t>::max()}
  {
  }

  BehResult update(flecs::world &world, flecs::entity entity, Blackboard &blackboard) override
  {
    std::vector<std::pair<size_t, float>> indexed_score{};
    indexed_score.reserve(nodes_.size());

    for (size_t index = 0; index < nodes_.size(); ++index)
    {
      auto score = nodes_[index].utility(blackboard) + nodes_inertia_[index];
      indexed_score.push_back(std::make_pair(index, score));
    }

    std::sort(indexed_score.begin(), indexed_score.end(), [](auto &lhs, auto &rhs) { return lhs.second > rhs.second; });

    for (const auto& [index, score] : indexed_score)
    {
      if (auto result = nodes_[index].node->update(world, entity, blackboard); result != BEH_FAIL)
      {
        if (index != previously_selected_node_index_)
        {
          nodes_inertia_[index] += inertia_ + inertia_decrease_rate_; // add inertia_decrease_rate_ to skip decreasing for the first time
        }
        for (auto& node_inertia : nodes_inertia_)
        {
          node_inertia -= inertia_decrease_rate_;
        }
        previously_selected_node_index_ = index;

        return result;
      }
    }

    return BEH_FAIL;
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

struct PatchUp : public BehNode
{
  float hpThreshold = 100.f;
  PatchUp(float threshold) : hpThreshold(threshold) {}

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &) override
  {
    BehResult res = BEH_SUCCESS;
    entity.set([&](Action &a, Hitpoints &hp)
    {
      if (hp.hitpoints >= hpThreshold)
        return;
      res = BEH_RUNNING;
      a.action = EA_HEAL_SELF;
    });
    return res;
  }
};



BehNode *sequence(const std::vector<BehNode*> &nodes)
{
  Sequence *seq = new Sequence;
  for (BehNode *node : nodes)
    seq->pushNode(node);
  return seq;
}

BehNode *selector(const std::vector<BehNode*> &nodes)
{
  Selector *sel = new Selector;
  for (BehNode *node : nodes)
    sel->pushNode(node);
  return sel;
}

BehNode *utility_selector(const std::vector<std::pair<BehNode*, Utility>> &nodes)
{
  return new UtilitySelector{nodes};
}

BehNode *weighted_random_utility_selector(const std::vector<std::pair<BehNode*, Utility>> &nodes)
{
  return new WeightedRandomUtilitySelector{nodes};
}

BehNode *inertail_utility_selector(const std::vector<std::pair<BehNode*, Utility>> &nodes)
{
  return new InertialUtilitySelector{nodes};
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

BehNode *patch_up(float thres)
{
  return new PatchUp(thres);
}


