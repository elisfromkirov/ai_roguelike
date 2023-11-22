#include "goapPlanner.h"
#include "goapWorldState.h"
#include <algorithm>
#include <cfloat>
#include <limits>

struct PlanNode
{
  goap::WorldState worldState;
  goap::WorldState prevState;

  float g = 0;
  float h = 0;

  size_t actionId;

  const goap::WorldState& get_state() const
  {
    return worldState;
  }

  float get_g_score() const
  {
    return g;
  }

  float get_f_score() const
  {
    return g + h;
  }

  size_t get_action_id() const
  {
    return actionId;
  }
};

constexpr auto InvalidActionId = std::numeric_limits<size_t>::max();

static float heuristic(const goap::WorldState &from, const goap::WorldState &to)
{
  float cost = 0;
  for (size_t i = 0; i < to.size(); ++i)
    if (to[i] >= 0) // we care about it
      cost += float(abs(to[i] - from[i]));
  return cost;
}

static void reconstruct_plan(PlanNode &goal_node, const std::vector<PlanNode> &closed, std::vector<goap::PlanStep> &plan)
{
  PlanNode &curNode = goal_node;
  while (curNode.actionId != size_t(-1))
  {
    plan.push_back({curNode.actionId, curNode.worldState});
    auto itf = std::find_if(closed.begin(), closed.end(), [&](const PlanNode &n) { return n.worldState == curNode.prevState; });
    curNode = *itf;
  }
  std::reverse(plan.begin(), plan.end());
}

float goap::make_plan(const Planner &planner, const WorldState &from, const WorldState &to, std::vector<PlanStep> &plan)
{
  std::vector<PlanNode> openList = {PlanNode{from, from, 0, heuristic(from, to), size_t(-1)}};
  std::vector<PlanNode> closedList = {};
  while (!openList.empty())
  {
    auto minIt = openList.begin();
    float minF = minIt->g + minIt->h;
    for (auto it = openList.begin(); it != openList.end(); ++it)
      if (it->g + it->h < minF)
      {
        minF = it->g + it->h;
        minIt = it;
      }
    PlanNode cur = *minIt;
    openList.erase(minIt);
    if (heuristic(cur.worldState, to) == 0) // we've reached our goal
    {
      reconstruct_plan(cur, closedList, plan);
      return minF;
    }
    closedList.push_back(cur);
    std::vector<size_t> transitions = find_valid_state_transitions(planner, cur.worldState);
    //const bool firstIter = openList.empty();
    //printf("------------\n");
    for (size_t actId : transitions)
    {
      //printf("valid action: %s\n", planner.actions[actId].name.c_str());
      WorldState st = apply_action(planner, actId, cur.worldState);
      const float score = cur.g + get_action_cost(planner, actId);
      auto openIt = std::find_if(openList.begin(), openList.end(), [&](const PlanNode &n) { return st == n.worldState; });
      auto closeIt = std::find_if(closedList.begin(), closedList.end(), [&](const PlanNode &n) { return st == n.worldState; });
      if (openIt != openList.end() && score < openIt->g)
      {
        openIt->g = score;
        openIt->prevState = cur.worldState;
      }
      if (closeIt != closedList.end() && score < closeIt->g)
      {
        closeIt->g = score;
        closeIt->prevState = cur.worldState;
      }
      if (closeIt == closedList.end() && openIt == openList.end())
        openList.push_back({st, cur.worldState, score, heuristic(st, to), actId});
    }
  }
  return 0.f;
}

void goap::print_plan(const Planner &planner, const WorldState &init, const std::vector<PlanStep> &plan)
{
  printf("%15s: ", "");
  std::vector<int> dlen;
  for (size_t i = 0; i < planner.wdesc.size(); ++i)
  {
    // print names by searching
    for (auto it : planner.wdesc)
    {
      if (it.second == i)
      {
        printf("|%s|", it.first.c_str());
        dlen.push_back(int(it.first.size()));
        break;
      }
    }
  }
  printf("\n");
  printf("%15s: ", "");
  for (size_t i = 0; i < init.size(); ++i)
    printf("|%*d|", dlen[i], init[i]);
  printf("\n");
  for (const PlanStep &step : plan)
  {
    printf("%15s: ", planner.actions[step.action].name.c_str());
    for (size_t i = 0; i < step.worldState.size(); ++i)
      printf("|%*d|", dlen[i], step.worldState[i]);
    printf("\n");
  }
}

namespace goap
{

constexpr bool is_inf(const float val)
{
  return fabs(val - 100) < FLT_EPSILON;
}

struct SearchResult
{
  float f_score;
  bool found;
};

bool is_state_in_path(const std::vector<PlanNode>& path, const WorldState& state)
{
  for (const auto& node : path)
  {
    if (node.get_state() == state)
    {
      return true;
    }
  }
  return false;
}

SearchResult search(const Planner& planner, const WorldState& target, std::vector<PlanNode>& path, float min_f_score)
{
  const auto& node = path.back();

  if (node.get_f_score() > min_f_score)
  {
    return {node.get_f_score(), false};
  }

  if (heuristic(node.get_state(), target) == 0)
  {
    return {0.f, true};
  }

  auto next_min_f_score = FLT_MAX;
  auto next_action_ids = find_valid_state_transitions(planner, node.get_state());
  for (auto next_action_id : next_action_ids)
  {
    auto next_state = apply_action(planner, next_action_id, path.back().get_state());

    if (!is_state_in_path(path, next_state))
    {
      auto g_score = path.back().get_g_score() + get_action_cost(planner, next_action_id);
      auto h_score = heuristic(next_state, target);

      path.push_back({next_state, path.back().get_state(), g_score, h_score, next_action_id});

      auto result = search(planner, target, path, min_f_score);
      if (result.found)
      {
        return result;
      }
      else if (next_min_f_score > result.f_score)
      {
        next_min_f_score = result.f_score;
      }

      path.pop_back();
    }
  }
  return {next_min_f_score, false};
}

std::vector<PlanNode> find_path(const Planner& planner, const WorldState& source, const WorldState& target)
{
  std::vector<PlanNode> path{PlanNode{source, {}, 0.f, heuristic(source, target), InvalidActionId}};

  auto min_f_score = path.back().get_f_score();

  while(true)
  {
    auto result = search(planner, target, path, min_f_score);

    if (result.found)
    {
      return path;
    }
    else if (is_inf(result.f_score))
    {
      return std::vector<PlanNode>{};
    }
    else
    {
      min_f_score = result.f_score;
    }
  }
}

float make_plan_ida_star(const Planner& planner, const WorldState& source, const WorldState& target, std::vector<PlanStep>& plan)
{
  auto path = find_path(planner, source, target);
  if (path.empty())
  {
    return FLT_MAX;
  }
  for (const auto& node : path)
  {
    if (node.get_action_id() == InvalidActionId)
    {
      continue;
    }
    plan.push_back({node.get_action_id(), node.get_state()});
  }
  return path.back().get_g_score();
}

}