#pragma once

#include "math.h"

#include <flecs.h>

#include <vector>

struct PortalConnection
{
  size_t connIdx;
  float score;

  size_t get_index() const { return connIdx; }
  float  get_score() const { return score;   }
};

struct PathPortal
{
  size_t startX, startY;
  size_t endX, endY;
  std::vector<PortalConnection> conns;

  IVec2 to_tile() const { return {static_cast<int>(startX + endX) / 2, static_cast<int>(startY + endY) / 2}; }
};

struct DungeonPortals
{
  size_t tileSplit;
  std::vector<PathPortal> portals;
  std::vector<std::vector<size_t>> tilePortalsIndices;
};

void prebuild_map(flecs::world &ecs);

struct DungeonData;

std::vector<IVec2> find_path_hierarchical(const DungeonData& data, const DungeonPortals& portals, IVec2 source, IVec2 target);
