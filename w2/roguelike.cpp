#include "roguelike.h"
#include "behaviourTree.h"
#include "ecsTypes.h"
#include "raylib.h"
#include "stateMachine.h"
#include "aiLibrary.h"
#include "blackboard.h"
#include <limits>

static void attach_gatherer_behaviour(flecs::entity entity)
{
  constexpr auto enemy_to_attack = "enemy_to_attack";
  constexpr auto buff_to_gather = "buff_to_gather";

  entity.add<BuffConsumerTag>();

  entity.set(Blackboard{});

  auto root =
    selector({
      sequence({
        find_enemy(entity, 2.f, enemy_to_attack),
        move_to_entity(entity, enemy_to_attack)
      }),
      sequence({
        find_nearest_buff(entity, buff_to_gather),
        move_to_entity(entity, buff_to_gather)
      }),
      sequence({
        find_enemy(entity, std::numeric_limits<float>::max(), enemy_to_attack),
        move_to_entity(entity, enemy_to_attack),
      })
  });

  entity.set(BehaviourTree{root});
}

static flecs::entity create_way_point(flecs::world &world, const Position& position, Color color) {
  return world.entity()
    .set(position)
    .set(Color{color});
}

static void attach_guard_behaviour(flecs::entity entity, flecs::world &world, const std::vector<Position>& positions, Color color)
{
  constexpr auto enemy_to_attack = "enemy_to_attack";
  constexpr auto way_point = "way_point";

  std::vector<flecs::entity> way_points{};
  for (const auto& position : positions)
  {
    way_points.push_back(create_way_point(world, position, color));
  }
  for (size_t i = 0; i < positions.size(); ++i)
  {
    way_points[i].set(WayPoint{way_points[(i + 1) % positions.size()]});
  }

  entity.set(Blackboard{});

  auto root =
    selector({
      sequence({
        find_enemy(entity, 2.f, enemy_to_attack),
        move_to_entity(entity, enemy_to_attack)
      }),
      sequence({
        move_to_entity(entity, way_point),
        find_next_way_point(entity, way_points[0], way_point)
      }),
    });

  entity.set(BehaviourTree{root});
}

static void create_minotaur_beh(flecs::entity e)
{
  e.set(Blackboard{});
  BehNode *root =
    selector({
      sequence({
        is_low_hp(50.f),
        find_enemy(e, 4.f, "flee_enemy"),
        flee(e, "flee_enemy")
      }),
      sequence({
        find_enemy(e, 3.f, "attack_enemy"),
        move_to_entity(e, "attack_enemy")
      }),
      patrol(e, 2.f, "patrol_pos")
    });
  e.set(BehaviourTree{root});
}

static flecs::entity create_monster(flecs::world &ecs, int x, int y, Color col, const char *texture_src)
{
  flecs::entity textureSrc = ecs.entity(texture_src);
  return ecs.entity()
    .set(Position{x, y})
    .set(MovePos{x, y})
    .set(Hitpoints{100.f})
    .set(Action{EA_NOP})
    .set(Color{col})
    .add<TextureSource>(textureSrc)
    .set(StateMachine{})
    .set(Team{1})
    .set(NumActions{1, 0})
    .set(MeleeDamage{20.f})
    .set(Blackboard{});
}

static void create_player(flecs::world &ecs, int x, int y, const char *texture_src)
{
  flecs::entity textureSrc = ecs.entity(texture_src);
  ecs.entity("player")
    .set(Position{x, y})
    .set(MovePos{x, y})
    .set(Hitpoints{100.f})
    //.set(Color{0xee, 0xee, 0xee, 0xff})
    .set(Action{EA_NOP})
    .add<IsPlayer>()
    .set(Team{0})
    .set(PlayerInput{})
    .set(NumActions{2, 0})
    .set(Color{255, 255, 255, 255})
    .add<TextureSource>(textureSrc)
    .set(MeleeDamage{50.f})
    .add<BuffConsumerTag>();
}

static void create_heal(flecs::world &ecs, int x, int y, float amount)
{
  ecs.entity()
    .add<BuffTag>()
    .set(Position{x, y})
    .set(HealAmount{amount})
    .set(Color{0xff, 0x44, 0x44, 0xff});
}

static void create_powerup(flecs::world &ecs, int x, int y, float amount)
{
  ecs.entity()
    .add<BuffTag>()
    .set(Position{x, y})
    .set(PowerupAmount{amount})
    .set(Color{0xff, 0xff, 0x00, 0xff});
}

static void register_roguelike_systems(flecs::world &ecs)
{
  ecs.system<PlayerInput, Action, const IsPlayer>()
    .each([&](PlayerInput &inp, Action &a, const IsPlayer)
    {
      bool left = IsKeyDown(KEY_LEFT);
      bool right = IsKeyDown(KEY_RIGHT);
      bool up = IsKeyDown(KEY_UP);
      bool down = IsKeyDown(KEY_DOWN);
      if (left && !inp.left)
        a.action = EA_MOVE_LEFT;
      if (right && !inp.right)
        a.action = EA_MOVE_RIGHT;
      if (up && !inp.up)
        a.action = EA_MOVE_UP;
      if (down && !inp.down)
        a.action = EA_MOVE_DOWN;
      inp.left = left;
      inp.right = right;
      inp.up = up;
      inp.down = down;
    });
  ecs.system<const Position, const Color>()
    .term<TextureSource>(flecs::Wildcard).not_()
    .each([&](const Position &pos, const Color color)
    {
      const Rectangle rect = {float(pos.x), float(pos.y), 1, 1};
      DrawRectangleRec(rect, color);
    });
  ecs.system<const Position, const Color>()
    .term<TextureSource>(flecs::Wildcard)
    .each([&](flecs::entity e, const Position &pos, const Color color)
    {
      const auto textureSrc = e.target<TextureSource>();
      DrawTextureQuad(*textureSrc.get<Texture2D>(),
          Vector2{1, 1}, Vector2{0, 0},
          Rectangle{float(pos.x), float(pos.y), 1, 1}, color);
    });
}


void init_roguelike(flecs::world &ecs)
{
  register_roguelike_systems(ecs);

  ecs.entity("swordsman_tex")
    .set(Texture2D{LoadTexture("assets/swordsman.png")});
  ecs.entity("minotaur_tex")
    .set(Texture2D{LoadTexture("assets/minotaur.png")});

  ecs.observer<Texture2D>()
    .event(flecs::OnRemove)
    .each([](Texture2D texture)
      {
        UnloadTexture(texture);
      });

  create_player(ecs, 0, 0, "swordsman_tex");

  attach_gatherer_behaviour(create_monster(ecs, -5, 5, Color{0xff, 0x00, 0xff, 0xff}, "minotaur_tex"));

  create_powerup(ecs, -3, -5, 10.f);
  create_powerup(ecs, -7, 5, 10.f);

  create_heal(ecs, -5,  5, 50.f);
  create_heal(ecs, -6, -5, 50.f);

  attach_guard_behaviour(create_monster(ecs, 7, 0, Color{0x00, 0xff, 0xff, 0xff}, "minotaur_tex"), ecs, {Position{7, 5}, Position{7, -5}}, Color{0x00, 0xff, 0xff, 0xff});
}

static bool is_player_acted(flecs::world &ecs)
{
  static auto processPlayer = ecs.query<const IsPlayer, const Action>();
  bool playerActed = false;
  processPlayer.each([&](const IsPlayer, const Action &a)
  {
    playerActed = a.action != EA_NOP;
  });
  return playerActed;
}

static bool upd_player_actions_count(flecs::world &ecs)
{
  static auto updPlayerActions = ecs.query<const IsPlayer, NumActions>();
  bool actionsReached = false;
  updPlayerActions.each([&](const IsPlayer, NumActions &na)
  {
    na.curActions = (na.curActions + 1) % na.numActions;
    actionsReached |= na.curActions == 0;
  });
  return actionsReached;
}

static Position move_pos(Position pos, int action)
{
  if (action == EA_MOVE_LEFT)
    pos.x--;
  else if (action == EA_MOVE_RIGHT)
    pos.x++;
  else if (action == EA_MOVE_UP)
    pos.y--;
  else if (action == EA_MOVE_DOWN)
    pos.y++;
  return pos;
}

static void process_actions(flecs::world &ecs)
{
  static auto processActions = ecs.query<Action, Position, MovePos, const MeleeDamage, const Team>();
  static auto checkAttacks = ecs.query<const MovePos, Hitpoints, const Team>();
  // Process all actions
  ecs.defer([&]
  {
    processActions.each([&](flecs::entity entity, Action &a, Position &pos, MovePos &mpos, const MeleeDamage &dmg, const Team &team)
    {
      Position nextPos = move_pos(pos, a.action);
      bool blocked = false;
      checkAttacks.each([&](flecs::entity enemy, const MovePos &epos, Hitpoints &hp, const Team &enemy_team)
      {
        if (entity != enemy && epos == nextPos)
        {
          blocked = true;
          if (team.team != enemy_team.team)
            hp.hitpoints -= dmg.damage;
        }
      });
      if (blocked)
        a.action = EA_NOP;
      else
        mpos = nextPos;
    });
    // now move
    processActions.each([&](Action &a, Position &pos, MovePos &mpos, const MeleeDamage &, const Team&)
    {
      pos = mpos;
      a.action = EA_NOP;
    });
  });

  static auto deleteAllDead = ecs.query<const Hitpoints>();
  ecs.defer([&]
  {
    deleteAllDead.each([&](flecs::entity entity, const Hitpoints &hp)
    {
      if (hp.hitpoints <= 0.f)
        entity.destruct();
    });
  });

  static auto buffConsumers = ecs.query<const BuffConsumerTag, const Position, Hitpoints, MeleeDamage>();
  static auto healPickup = ecs.query<const Position, const HealAmount>();
  static auto powerupPickup = ecs.query<const Position, const PowerupAmount>();
  ecs.defer([&]
  {
    buffConsumers.each([&](const BuffConsumerTag&, const Position &pos, Hitpoints &hp, MeleeDamage &dmg)
    {
      healPickup.each([&](flecs::entity entity, const Position &ppos, const HealAmount &amt)
      {
        if (pos == ppos)
        {
          hp.hitpoints += amt.amount;
          entity.destruct();
        }
      });
      powerupPickup.each([&](flecs::entity entity, const Position &ppos, const PowerupAmount &amt)
      {
        if (pos == ppos)
        {
          dmg.damage += amt.amount;
          entity.destruct();
        }
      });
    });
  });
}

void process_turn(flecs::world &ecs)
{
  static auto stateMachineAct = ecs.query<StateMachine>();
  static auto behTreeUpdate = ecs.query<BehaviourTree, Blackboard>();
  if (is_player_acted(ecs))
  {
    if (upd_player_actions_count(ecs))
    {
      // Plan action for NPCs
      ecs.defer([&]
      {
        stateMachineAct.each([&](flecs::entity e, StateMachine &sm)
        {
          sm.act(0.f, ecs, e);
        });
        behTreeUpdate.each([&](flecs::entity e, BehaviourTree &bt, Blackboard &bb)
        {
          bt.update(ecs, e, bb);
        });
      });
    }
    process_actions(ecs);
  }
}

void print_stats(flecs::world &ecs)
{
  static auto playerStatsQuery = ecs.query<const IsPlayer, const Hitpoints, const MeleeDamage>();
  playerStatsQuery.each([&](const IsPlayer &, const Hitpoints &hp, const MeleeDamage &dmg)
  {
    DrawText(TextFormat("hp: %d", int(hp.hitpoints)), 20, 20, 20, WHITE);
    DrawText(TextFormat("power: %d", int(dmg.damage)), 20, 40, 20, WHITE);
  });
}

