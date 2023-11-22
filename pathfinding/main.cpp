#include "raylib.h"
#include <functional>
#include <vector>
#include <limits>
#include <float.h>
#include <cmath>
#include "math.h"
#include "dungeonGen.h"
#include "dungeonUtils.h"
#include <queue>
#include <set>

template<typename T>
static size_t coord_to_idx(T x, T y, size_t w)
{
  return size_t(y) * w + size_t(x);
}

static void draw_nav_grid(const char *input, size_t width, size_t height)
{
  for (size_t y = 0; y < height; ++y)
    for (size_t x = 0; x < width; ++x)
    {
      char symb = input[coord_to_idx(x, y, width)];
      Color color = GetColor(symb == ' ' ? 0xeeeeeeff : symb == 'o' ? 0x7777ffff : 0x222222ff);
      const Rectangle rect = {float(x), float(y), 1.f, 1.f};
      DrawRectangleRec(rect, color);
      //DrawPixel(int(x), int(y), color);
    }
}

static void draw_path(std::vector<Position> path)
{
  for (const Position &p : path)
  {
    const Rectangle rect = {float(p.x), float(p.y), 1.f, 1.f};
    DrawRectangleRec(rect, GetColor(0x44000088));
  }
}

static std::vector<Position> reconstruct_path(std::vector<Position> prev, Position to, size_t width)
{
  Position curPos = to;
  std::vector<Position> res = {curPos};
  while (prev[coord_to_idx(curPos.x, curPos.y, width)] != Position{-1, -1})
  {
    curPos = prev[coord_to_idx(curPos.x, curPos.y, width)];
    res.insert(res.begin(), curPos);
  }
  return res;
}

float heuristic(Position lhs, Position rhs)
{
  return sqrtf(square(float(lhs.x - rhs.x)) + square(float(lhs.y - rhs.y)));
};

static float ida_star_search(const char *input, size_t width, size_t height, std::vector<Position> &path, const float g, const float bound, Position to)
{
  const Position &p = path.back();
  const float f = g + heuristic(p, to);
  if (f > bound)
    return f;
  if (p == to)
    return -f;
  float min = FLT_MAX;
  auto checkNeighbour = [&](Position p) -> float
  {
    // out of bounds
    if (p.x < 0 || p.y < 0 || p.x >= int(width) || p.y >= int(height))
      return 0.f;
    size_t idx = coord_to_idx(p.x, p.y, width);
    // not empty
    if (input[idx] == '#')
      return 0.f;
    if (std::find(path.begin(), path.end(), p) != path.end())
      return 0.f;
    path.push_back(p);
    float weight = input[idx] == 'o' ? 10.f : 1.f;
    float gScore = g + 1.f * weight; // we're exactly 1 unit away
    const float t = ida_star_search(input, width, height, path, gScore, bound, to);
    if (t < 0.f)
      return t;
    if (t < min)
      min = t;
    path.pop_back();
    return t;
  };
  float lv = checkNeighbour({p.x + 1, p.y + 0});
  if (lv < 0.f) return lv;
  float rv = checkNeighbour({p.x - 1, p.y + 0});
  if (rv < 0.f) return rv;
  float tv = checkNeighbour({p.x + 0, p.y + 1});
  if (tv < 0.f) return tv;
  float bv = checkNeighbour({p.x + 0, p.y - 1});
  if (bv < 0.f) return bv;
  return min;
}

static std::vector<Position> find_ida_star_path(const char *input, size_t width, size_t height, Position from, Position to)
{
  float bound = heuristic(from, to);
  std::vector<Position> path = {from};
  while (true)
  {
    const float t = ida_star_search(input, width, height, path, 0.f, bound, to);
    if (t < 0.f)
      return path;
    if (t == FLT_MAX)
      return {};
    bound = t;
    printf("new bound %0.1f\n", bound);
  }
  return {};
}

static std::vector<Position> find_path_a_star(const char *input, size_t width, size_t height, Position from, Position to, float weight)
{
  if (from.x < 0 || from.y < 0 || from.x >= int(width) || from.y >= int(height))
    return std::vector<Position>();
  size_t inpSize = width * height;

  std::vector<float> g(inpSize, std::numeric_limits<float>::max());
  std::vector<float> f(inpSize, std::numeric_limits<float>::max());
  std::vector<Position> prev(inpSize, {-1,-1});

  auto getG = [&](Position p) -> float { return g[coord_to_idx(p.x, p.y, width)]; };
  auto getF = [&](Position p) -> float { return f[coord_to_idx(p.x, p.y, width)]; };

  g[coord_to_idx(from.x, from.y, width)] = 0;
  f[coord_to_idx(from.x, from.y, width)] = weight * heuristic(from, to);

  std::vector<Position> openList = {from};
  std::vector<Position> closedList;

  while (!openList.empty())
  {
    size_t bestIdx = 0;
    float bestScore = getF(openList[0]);
    for (size_t i = 1; i < openList.size(); ++i)
    {
      float score = getF(openList[i]);
      if (score < bestScore)
      {
        bestIdx = i;
        bestScore = score;
      }
    }
    if (openList[bestIdx] == to)
      return reconstruct_path(prev, to, width);
    Position curPos = openList[bestIdx];
    openList.erase(openList.begin() + bestIdx);
    if (std::find(closedList.begin(), closedList.end(), curPos) != closedList.end())
      continue;
    size_t idx = coord_to_idx(curPos.x, curPos.y, width);
    const Rectangle rect = {float(curPos.x), float(curPos.y), 1.f, 1.f};
    DrawRectangleRec(rect, Color{uint8_t(g[idx]), uint8_t(g[idx]), 0, 100});
    closedList.emplace_back(curPos);
    auto checkNeighbour = [&](Position p)
    {
      // out of bounds
      if (p.x < 0 || p.y < 0 || p.x >= int(width) || p.y >= int(height))
        return;
      size_t idx = coord_to_idx(p.x, p.y, width);
      // not empty
      if (input[idx] == '#')
        return;
      float edgeWeight = input[idx] == 'o' ? 10.f : 1.f;
      float gScore = getG(curPos) + 1.f * edgeWeight; // we're exactly 1 unit away
      if (gScore < getG(p))
      {
        prev[idx] = curPos;
        g[idx] = gScore;
        f[idx] = gScore + weight * heuristic(p, to);
      }
      bool found = std::find(openList.begin(), openList.end(), p) != openList.end();
      if (!found)
        openList.emplace_back(p);
    };
    checkNeighbour({curPos.x + 1, curPos.y + 0});
    checkNeighbour({curPos.x - 1, curPos.y + 0});
    checkNeighbour({curPos.x + 0, curPos.y + 1});
    checkNeighbour({curPos.x + 0, curPos.y - 1});
  }
  // empty path
  return std::vector<Position>();
}

constexpr bool is_inf(const float val)
{
  return fabs(val - FLT_MAX) < FLT_EPSILON;
}

using Weight = float;

class Map
{
public:
  explicit Map(const char *map, std::size_t width, std::size_t height)
    : underlying_map_(map)
    , width_(width)
    , height_(height)
  {
  }

  float get_weight(std::size_t index) const
  {
    if (underlying_map_[index] == '#')
    {
      return FLT_MAX;
    }
    if (underlying_map_[index] == 'o')
    {
      return 10.f;
    }
    return 1.f;
  }

  std::size_t get_width() const
  {
    return width_;
  }

  std::size_t get_height() const
  {
    return height_;
  }

private:
  const char *underlying_map_;
  std::size_t width_;
  std::size_t height_;
};

class PathFinder
{
  class PositionComparer
  {
  public:
    using size_type = std::size_t;
    using value_type = Position;
    using reference = value_type&;
    using const_reference = const value_type&;
    using pointer = value_type*;
    using const_pointer = const value_type*;


  public:
    explicit PositionComparer(PathFinder& path_finder)
      : path_finder_(path_finder)
    {
    }

    bool operator()(const Position &lhs, const Position &rhs) const
    {
      return path_finder_.get_f_score(rhs) < path_finder_.get_f_score(lhs);
    }

  private:
    PathFinder& path_finder_;
  };

public:
  PathFinder(const char *map, std::size_t width, std::size_t height, bool enable_visualisation = true)
    : map_(map, width, height)
    , g_score_(width * height, FLT_MAX)
    , prev_(width * height, Position{})
    , open_(PositionComparer(*this))
    , closed_()
    , incons_()
    , source_()
    , target_()
    , epsilon_()
    , enable_visualisation_(enable_visualisation)
  {
  }

  std::vector<Position> find_path(const Position &source, const Position &target, const float epsilon = 5.f, const float step = 0.5f)
  {
    reset(source, target, epsilon);

    improve_path();

    while (estimate_epsilon_hatch() > 1.f)
    {
      epsilon_ -= step;

      // copy open_ positions to incons and then rebuild a queue, because epsilon has been changed
      while (!open_.empty())
      {
        incons_.push_back(open_.top());
        open_.pop();
      }
      while (!incons_.empty())
      {
        open_.push(incons_.back());
        incons_.pop_back();
      }

      closed_.clear();

      improve_path();
    }

    return reconstruct_path();
  }

  Weight get_f_score(const Position &position) const
  {
    return g_score_[position_to_index(position)] + epsilon_ * heuristic(position, target_);
  }

private:
  void reset(const Position &source, const Position &target, const float epsilon)
  {
    std::fill(std::begin(g_score_), std::end(g_score_), FLT_MAX);
    g_score_[position_to_index(source)] = 0.f;

    std::fill(std::begin(prev_), std::end(prev_), Position{});

    while (!open_.empty())
    {
      open_.pop();
    }
    open_.push(source);

    closed_.clear();

    incons_.clear();

    source_ = source;
    target_ = target;

    epsilon_ = epsilon;
  }

  std::size_t position_to_index(const Position &position) const
  {
    return static_cast<std::size_t>(position.x) + static_cast<std::size_t>(position.y) * map_.get_width();
  }

  void improve_path()
  {
    while (!open_.empty() && get_f_score(target_) > get_f_score(open_.top()))
    {
      const auto position = open_.top();
      open_.pop();

      if (enable_visualisation_)
      {
        const Rectangle rect = {float(position.x), float(position.y), 1.f, 1.f};
        DrawRectangleRec(rect, GetColor(0xbbbbbbff));
      }

      closed_.insert(position);

      improve_neighbour(position, Position{position.x + 1, position.y + 0});
      improve_neighbour(position, Position{position.x - 1, position.y + 0});
      improve_neighbour(position, Position{position.x + 0, position.y + 1});
      improve_neighbour(position, Position{position.x + 0, position.y - 1});
    }
  }

  void improve_neighbour(const Position &position, const Position &neighbour_position)
  {
    if (neighbour_position.x < 0 || static_cast<std::size_t>(neighbour_position.x) >= map_.get_width())
    {
      return;
    }

    if (neighbour_position.y < 0 || static_cast<std::size_t>(neighbour_position.y) >= map_.get_height())
    {
      return;
    }

    auto g_score = g_score_[position_to_index(position)] + map_.get_weight(position_to_index(neighbour_position));
    if (g_score_[position_to_index(neighbour_position)] > g_score)
    {
      g_score_[position_to_index(neighbour_position)] = g_score;
      prev_[position_to_index(neighbour_position)] = position;

      if (closed_.contains(neighbour_position))
      {
        incons_.push_back(neighbour_position);
      }
      else
      {
        open_.push(neighbour_position);
      }
    }
  }

  Weight estimate_epsilon_hatch()
  {
    auto min_epsilon = FLT_MAX;
    if (!open_.empty())
    {
      min_epsilon = std::min(min_epsilon, get_f_score(open_.top()));
    }
    for (const auto &position : incons_)
    {
      min_epsilon = std::min(min_epsilon, get_f_score(position));
    }

    if (is_inf(min_epsilon) || is_inf(g_score_[position_to_index(target_)]))
    {
      return epsilon_;
    }
    return std::min(epsilon_, g_score_[position_to_index(target_)] / min_epsilon);
  }

  std::vector<Position> reconstruct_path()
  {
    if (is_inf(g_score_[position_to_index(target_)]))
    {
      return {};
    }

    std::vector<Position> path{};
    auto prev = target_;
    while (true)
    {
      path.push_back(prev);
      prev = prev_[position_to_index(prev)];
      if (prev == source_)
      {
        path.push_back(source_);
        return path;
      }
    }
  }

private:
  Map map_;
  std::vector<Weight> g_score_; ///< Map from index associated with position to minimal weight of path from source position to position
  std::vector<Position> prev_;
  std::priority_queue<Position, std::vector<Position>, PositionComparer> open_;
  std::set<Position> closed_;
  std::vector<Position> incons_;
  Position source_;
  Position target_;
  float epsilon_;
  bool enable_visualisation_;
};

void draw_nav_data(const char *input, size_t width, size_t height, Position from, Position to, float weight)
{
  draw_nav_grid(input, width, height);

  PathFinder path_finder{input, width, height};
  std::vector<Position> path = path_finder.find_path(from, to);
  // std::vector<Position> path = find_path_a_star(input, width, height, from, to, weight);
  // std::vector<Position> path = find_ida_star_path(input, width, height, from, to);
  draw_path(path);
}

int main(int /*argc*/, const char ** /*argv*/)
{
  int width = 1920;
  int height = 1080;
  InitWindow(width, height, "w3 AI MIPT");

  const int scrWidth = GetMonitorWidth(0);
  const int scrHeight = GetMonitorHeight(0);
  if (scrWidth < width || scrHeight < height)
  {
    width = std::min(scrWidth, width);
    height = std::min(scrHeight - 150, height);
    SetWindowSize(width, height);
  }

  constexpr size_t dungWidth = 100;
  constexpr size_t dungHeight = 100;
  char *navGrid = new char[dungWidth * dungHeight];
  gen_drunk_dungeon(navGrid, dungWidth, dungHeight, 24, 100);
  spill_drunk_water(navGrid, dungWidth, dungHeight, 8, 10);
  float weight = 1.f;

  Position from = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
  Position to = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);

  Camera2D camera = { {0, 0}, {0, 0}, 0.f, 1.f };
  //camera.offset = Vector2{ width * 0.5f, height * 0.5f };
  camera.zoom = float(height) / float(dungHeight);

  SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
  while (!WindowShouldClose())
  {
    // pick pos
    Vector2 mousePosition = GetScreenToWorld2D(GetMousePosition(), camera);
    Position p{int(mousePosition.x), int(mousePosition.y)};
    if (IsMouseButtonPressed(2) || IsKeyPressed(KEY_Q))
    {
      size_t idx = coord_to_idx(p.x, p.y, dungWidth);
      if (idx < dungWidth * dungHeight)
        navGrid[idx] = navGrid[idx] == ' ' ? '#' : navGrid[idx] == '#' ? 'o' : ' ';
    }
    else if (IsMouseButtonPressed(0))
    {
      Position &target = from;
      target = p;
    }
    else if (IsMouseButtonPressed(1))
    {
      Position &target = to;
      target = p;
    }
    if (IsKeyPressed(KEY_SPACE))
    {
      gen_drunk_dungeon(navGrid, dungWidth, dungHeight, 24, 100);
      spill_drunk_water(navGrid, dungWidth, dungHeight, 8, 10);
      from = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
      to = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
    }
    if (IsKeyPressed(KEY_UP))
    {
      weight += 0.1f;
      printf("new weight %f\n", weight);
    }
    if (IsKeyPressed(KEY_DOWN))
    {
      weight = std::max(1.f, weight - 0.1f);
      printf("new weight %f\n", weight);
    }
    BeginDrawing();
      ClearBackground(BLACK);
      BeginMode2D(camera);
        draw_nav_data(navGrid, dungWidth, dungHeight, from, to, weight);
      EndMode2D();
    EndDrawing();
  }
  CloseWindow();
  return 0;
}
