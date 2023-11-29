#include "dungeonUtils.h"
#include "ecsTypes.h"
#include "math.h"
#include "pathfinder.h"

#include <algorithm>
#include <cfloat>
#include <limits>
#include <queue>

constexpr auto kMegaTileSize = 10;

float heuristic(IVec2 lhs, IVec2 rhs)
{
  return sqrtf(sqr(float(lhs.x - rhs.x)) + sqr(float(lhs.y - rhs.y)));
};

template<typename T>
static size_t coord_to_idx(T x, T y, size_t w)
{
  return size_t(y) * w + size_t(x);
}

static std::vector<IVec2> reconstruct_path(std::vector<IVec2> prev, IVec2 to, size_t width)
{
  IVec2 curPos = to;
  std::vector<IVec2> res = {curPos};
  while (prev[coord_to_idx(curPos.x, curPos.y, width)] != IVec2{-1, -1})
  {
    curPos = prev[coord_to_idx(curPos.x, curPos.y, width)];
    res.insert(res.begin(), curPos);
  }
  return res;
}

static std::vector<IVec2> find_path_a_star(const DungeonData &dd, IVec2 from, IVec2 to,
                                           IVec2 lim_min, IVec2 lim_max)
{
  if (from.x < 0 || from.y < 0 || from.x >= int(dd.width) || from.y >= int(dd.height))
    return std::vector<IVec2>();
  size_t inpSize = dd.width * dd.height;

  std::vector<float> g(inpSize, std::numeric_limits<float>::max());
  std::vector<float> f(inpSize, std::numeric_limits<float>::max());
  std::vector<IVec2> prev(inpSize, {-1,-1});

  auto getG = [&](IVec2 p) -> float { return g[coord_to_idx(p.x, p.y, dd.width)]; };
  auto getF = [&](IVec2 p) -> float { return f[coord_to_idx(p.x, p.y, dd.width)]; };

  g[coord_to_idx(from.x, from.y, dd.width)] = 0;
  f[coord_to_idx(from.x, from.y, dd.width)] = heuristic(from, to);

  std::vector<IVec2> openList = {from};
  std::vector<IVec2> closedList;

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
      return reconstruct_path(prev, to, dd.width);
    IVec2 curPos = openList[bestIdx];
    openList.erase(openList.begin() + bestIdx);
    if (std::find(closedList.begin(), closedList.end(), curPos) != closedList.end())
      continue;
    size_t idx = coord_to_idx(curPos.x, curPos.y, dd.width);
    closedList.emplace_back(curPos);
    auto checkNeighbour = [&](IVec2 p)
    {
      // out of bounds
      if (p.x < lim_min.x || p.y < lim_min.y || p.x >= lim_max.x || p.y >= lim_max.y)
        return;
      size_t idx = coord_to_idx(p.x, p.y, dd.width);
      // not empty
      if (dd.tiles[idx] == dungeon::wall)
        return;
      float edgeWeight = 1.f;
      float gScore = getG(curPos) + 1.f * edgeWeight; // we're exactly 1 unit away
      if (gScore < getG(p))
      {
        prev[idx] = curPos;
        g[idx] = gScore;
        f[idx] = gScore + heuristic(p, to);
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
  return std::vector<IVec2>();
}


void prebuild_map(flecs::world &ecs)
{
  auto mapQuery = ecs.query<const DungeonData>();

  constexpr size_t splitTiles = 10;
  ecs.defer([&]()
  {
    mapQuery.each([&](flecs::entity e, const DungeonData &dd)
    {
      // go through each super tile
      const size_t width = dd.width / splitTiles;
      const size_t height = dd.height / splitTiles;

      auto check_border = [&](size_t xx, size_t yy,
                              size_t dir_x, size_t dir_y,
                              int offs_x, int offs_y,
                              std::vector<PathPortal> &portals)
      {
        int spanFrom = -1;
        int spanTo = -1;
        for (size_t i = 0; i < splitTiles; ++i)
        {
          size_t x = xx * splitTiles + i * dir_x;
          size_t y = yy * splitTiles + i * dir_y;
          size_t nx = x + offs_x;
          size_t ny = y + offs_y;
          if (dd.tiles[y * dd.width + x] != dungeon::wall &&
              dd.tiles[ny * dd.width + nx] != dungeon::wall)
          {
            if (spanFrom < 0)
              spanFrom = i;
            spanTo = i;
          }
          else if (spanFrom >= 0)
          {
            // write span
            portals.push_back({xx * splitTiles + spanFrom * dir_x + offs_x,
                               yy * splitTiles + spanFrom * dir_y + offs_y,
                               xx * splitTiles + spanTo * dir_x,
                               yy * splitTiles + spanTo * dir_y});
            spanFrom = -1;
          }
        }
        if (spanFrom >= 0)
        {
          portals.push_back({xx * splitTiles + spanFrom * dir_x + offs_x,
                             yy * splitTiles + spanFrom * dir_y + offs_y,
                             xx * splitTiles + spanTo * dir_x,
                             yy * splitTiles + spanTo * dir_y});
        }
      };

      std::vector<PathPortal> portals;
      std::vector<std::vector<size_t>> tilePortalsIndices;

      auto push_portals = [&](size_t x, size_t y,
                              int offs_x, int offs_y,
                              const std::vector<PathPortal> &new_portals)
      {
        for (const PathPortal &portal : new_portals)
        {
          size_t idx = portals.size();
          portals.push_back(portal);
          tilePortalsIndices[y * width + x].push_back(idx);
          tilePortalsIndices[(y + offs_y) * width + x + offs_x].push_back(idx);
        }
      };
      for (size_t y = 0; y < height; ++y)
        for (size_t x = 0; x < width; ++x)
        {
          tilePortalsIndices.push_back(std::vector<size_t>{});
          // check top
          if (y > 0)
          {
            std::vector<PathPortal> topPortals;
            check_border(x, y, 1, 0, 0, -1, topPortals);
            push_portals(x, y, 0, -1, topPortals);
          }
          // left
          if (x > 0)
          {
            std::vector<PathPortal> leftPortals;
            check_border(x, y, 0, 1, -1, 0, leftPortals);
            push_portals(x, y, -1, 0, leftPortals);
          }
        }
      for (size_t tidx = 0; tidx < tilePortalsIndices.size(); ++tidx)
      {
        const std::vector<size_t> &indices = tilePortalsIndices[tidx];
        size_t x = tidx % width;
        size_t y = tidx / width;
        IVec2 limMin{int((x + 0) * splitTiles), int((y + 0) * splitTiles)};
        IVec2 limMax{int((x + 1) * splitTiles), int((y + 1) * splitTiles)};
        for (size_t i = 0; i < indices.size(); ++i)
        {
          PathPortal &firstPortal = portals[indices[i]];
          for (size_t j = i + 1; j < indices.size(); ++j)
          {
            PathPortal &secondPortal = portals[indices[j]];
            // check path from i to j
            // check each position (to find closest dist) (could be made more optimal)
            bool noPath = false;
            size_t minDist = 0xffffffff;
            for (size_t fromY = std::max(firstPortal.startY, size_t(limMin.y));
                        fromY <= std::min(firstPortal.endY, size_t(limMax.y - 1)) && !noPath; ++fromY)
            {
              for (size_t fromX = std::max(firstPortal.startX, size_t(limMin.x));
                          fromX <= std::min(firstPortal.endX, size_t(limMax.x - 1)) && !noPath; ++fromX)
              {
                for (size_t toY = std::max(secondPortal.startY, size_t(limMin.y));
                            toY <= std::min(secondPortal.endY, size_t(limMax.y - 1)) && !noPath; ++toY)
                {
                  for (size_t toX = std::max(secondPortal.startX, size_t(limMin.x));
                              toX <= std::min(secondPortal.endX, size_t(limMax.x - 1)) && !noPath; ++toX)
                  {
                    IVec2 from{int(fromX), int(fromY)};
                    IVec2 to{int(toX), int(toY)};
                    std::vector<IVec2> path = find_path_a_star(dd, from, to, limMin, limMax);
                    if (path.empty() && from != to)
                    {
                      noPath = true; // if we found that there's no path at all - we can break out
                      break;
                    }
                    minDist = std::min(minDist, path.size());
                  }
                }
              }
            }
            // write pathable data and length
            if (noPath)
              continue;
            firstPortal.conns.push_back({indices[j], float(minDist)});
            secondPortal.conns.push_back({indices[i], float(minDist)});
          }
        }
      }
      e.set(DungeonPortals{splitTiles, portals, tilePortalsIndices});
    });
  });
}

bool tile_inside_dungeon(const DungeonData& data, IVec2 tile)
{
  return 0 <= tile.x && 0 <= tile.y && tile.x < static_cast<int>(data.width) && tile.y < static_cast<int>(data.height);
}

std::vector<float> make_scores(const DungeonData& data, const DungeonPortals& portals, IVec2 tile)
{
  std::vector<float> scores{};

  auto mega_tile = IVec2{tile.x / kMegaTileSize, tile.y / kMegaTileSize};

  auto left_down = IVec2{mega_tile.x * kMegaTileSize, mega_tile.y * kMegaTileSize};
  auto right_up = IVec2{(mega_tile.x + 1) * kMegaTileSize, (mega_tile.y + 1) * kMegaTileSize};

  for (const auto& portal : portals.portals)
  {
    auto portal_tile = portal.to_tile();
    auto portal_mega_tile = IVec2{portal_tile.x / kMegaTileSize, portal_tile.y / kMegaTileSize};
    if (mega_tile == portal_mega_tile)
    {
      auto path = find_path_a_star(data, tile, portal_tile, left_down, right_up);
      scores.push_back(path.size());
    }
    else
    {
      scores.push_back(std::numeric_limits<float>::max());
    }
  }

  return scores;
}

class Comparer
{
public:
  using size_type = size_t;
  using value_type = size_t;
  using reference = value_type&;
  using const_reference = const value_type&;
  using pointer = value_type*;
  using const_pointer = const value_type*;

public:
  Comparer(const std::vector<float>& g_score, const DungeonPortals& portals, IVec2 target, size_t target_index)
    : g_score_{g_score}
    , portals_{portals}
    , target_{target}
    , target_index_{target_index}
  {
  }

  bool operator()(const size_t lhs, const size_t rhs) const
  {
    auto lhs_f_score = 0.f;
    if (lhs != target_index_)
    {
      lhs_f_score = g_score_[lhs] + heuristic(portals_.portals[lhs].to_tile(), target_);
    }

    auto rhs_f_score = 0.f;
    if (rhs != target_index_)
    {
      rhs_f_score = g_score_[rhs] + heuristic(portals_.portals[rhs].to_tile(), target_);
    }

    return rhs_f_score < lhs_f_score;
  }

private:
  const std::vector<float>& g_score_;
  const DungeonPortals& portals_;
  const IVec2 target_;
  const size_t target_index_;
};

std::vector<IVec2> find_path_hierarchical(const DungeonData& data, const DungeonPortals& portals, IVec2 source, IVec2 target)
{
  if (!tile_inside_dungeon(data, source) || !tile_inside_dungeon(data, target))
  {
    return std::vector<IVec2>{};
  }

  if (source == target)
  {
    return std::vector<IVec2>{};
  }

  const auto source_mega_tile = IVec2{source.x / kMegaTileSize, source.y / kMegaTileSize};
  const auto target_mega_tile = IVec2{target.x / kMegaTileSize, target.y / kMegaTileSize};
  if (source_mega_tile == target_mega_tile)
  {
    const auto left_down = IVec2{source_mega_tile.x * kMegaTileSize, source_mega_tile.y * kMegaTileSize};
    const auto right_up = IVec2{(source_mega_tile.x + 1) * kMegaTileSize, (source_mega_tile.y + 1) * kMegaTileSize};
    const auto path = find_path_a_star(data,source, target, left_down, right_up);
    if (!path.empty())
    {
      return path;
    }
  }

  const auto source_scores = make_scores(data, portals, source); // score of path from source to i'th portal's center tile
  const auto target_scores = make_scores(data, portals, source); // score of path from i'th portal's center tile target to

  const auto source_index = portals.portals.size();
  const auto target_index = portals.portals.size() + 1;

  const auto total_indexes = portals.portals.size() + 2;

  std::vector<float> g_score(total_indexes, std::numeric_limits<float>::max());
  g_score[source_index] = 0;

  std::priority_queue<std::size_t, std::vector<std::size_t>, Comparer> open{Comparer{g_score, portals, target, target_index}};
  open.push(source_index);

  std::vector<bool> closed(total_indexes, false);

  std::vector<size_t> prev(total_indexes, total_indexes);

  while (!open.empty() && open.top() != target_index)
  {
    auto index = open.top();
    open.pop();
    closed[index] = true;

    if (index == source_index)
    {
      for (size_t i = 0; i < source_scores.size(); ++i)
      {
        if (!closed[i] && source_scores[i] < g_score[i])
        {
          g_score[i] = source_scores[i];
          prev[i] = source_index;
          open.push(i);
        }
      }
    }
    else
    {
      for (const auto& conn : portals.portals[index].conns)
      {
        auto tentative_g_score = g_score[conn.get_index()] + conn.get_score();
        if (!closed[conn.get_index()] || tentative_g_score < g_score[conn.get_index()])
        {
          g_score[conn.get_index()] = tentative_g_score;
          prev[conn.get_index()] = index;
          open.push(conn.get_index());
        }
      }
      if (target_scores[index] < g_score[target_index])
      {
        g_score[target_index] = target_scores[index];
        prev[target_index] = index;
        open.push(target_index);
      }
    }
  }

  if (open.empty() || open.top() != target_index)
  {
    return std::vector<IVec2>{};
  }

  std::vector<IVec2> path{};

  {
    const auto portal_tile = portals.portals[prev[target_index]].to_tile();

    const auto left_down = IVec2{std::min(target.x, portal_tile.x), std::min(target.y, portal_tile.y)};
    const auto right_up = IVec2{std::max(target.x, portal_tile.x) + 1, std::max(target.y, portal_tile.y) + 1};

    const auto intermediate_path = find_path_a_star(data, portal_tile, target, left_down, right_up);
    std::copy(std::begin(intermediate_path), std::end(intermediate_path), std::back_inserter(path));
  }

  auto index = prev[target_index];
  while (prev[index] != source_index)
  {
    const auto portal_tile = portals.portals[index].to_tile();
    const auto prev_portal_tile = portals.portals[prev[index]].to_tile();

    const auto left_down = IVec2{std::min(portal_tile.x, prev_portal_tile.x), std::min(portal_tile.y, prev_portal_tile.y)};
    const auto right_up = IVec2{std::max(portal_tile.x, prev_portal_tile.x) + 1, std::max(portal_tile.y, prev_portal_tile.y) + 1};

    const auto intermediate_path = find_path_a_star(data, prev_portal_tile, portal_tile, left_down, right_up);
    std::copy(std::begin(intermediate_path), std::end(intermediate_path), std::back_inserter(path));

    index = prev[index];
  }

  {
    const auto portal_tile = portals.portals[index].to_tile();

    const auto left_down = IVec2{std::min(source.x, portal_tile.x), std::min(source.y, portal_tile.y)};
    const auto right_up = IVec2{std::max(source.x, portal_tile.x) + 1, std::max(source.y, portal_tile.y) + 1};

    const auto intermediate_path = find_path_a_star(data, source, portal_tile, left_down, right_up);
    std::copy(std::begin(intermediate_path), std::end(intermediate_path), std::back_inserter(path));
  }

  return path;
}
