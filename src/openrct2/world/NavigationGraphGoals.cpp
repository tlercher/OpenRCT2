/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "NavigationGraphGoals.h"

#include "../GameState.h"
#include "../peep/GuestPathfinding.h"
#include "../ride/RideData.h"
#include "../ride/RideManager.hpp"
#include "Map.h"
#include "TileElementsView.h"
#include "tile_element/EntranceElement.h"
#include "tile_element/TrackElement.h"

#include <algorithm>
#include <queue>
#include <utility>

namespace OpenRCT2::Navigation
{
    namespace
    {
        // Populated by RefreshGoalNodesAndGraph, consumed by ResolveGoalSeedNodes. Shops don't store
        // their own tile location anywhere on the Ride/RideStation structs (unlike ride entrances/
        // exits), so this is built by the same full-map scan that computes goal terminals, rather
        // than re-scanning per goal-table build.
        std::unordered_map<RideId::UnderlyingType, TileCoordsXYZ> gShopTileByRide;

        // A ride entrance/exit's stored direction is the direction a peep must be travelling to walk
        // *into* it (matches GuestPathfinding.cpp:806-807's `direction == testEdge` check). The
        // approach tile is one step behind that, in the reverse direction - but a peep standing AT
        // the approach tile walks `facing` (not its reverse) to reach the goal, so the two directions
        // serve different purposes: this one only offsets the approach tile from the goal.
        Direction ApproachOffsetFor(Direction facing)
        {
            return DirectionReverse(facing);
        }

        // Scans the whole map once for every ride entrance/exit, shop track tile, park entrance, and
        // peep spawn, producing both the NavigationGraph inputs (forced nodes + goal terminals) and
        // the shop-tile index used later for goal-table seed resolution.
        void CollectGoals(
            const GameState_t& gameState, std::unordered_set<TileCoordsXYZ, TileCoordsXYZHash>& forcedNodeLocations,
            std::vector<NavGoalTerminal>& terminals)
        {
            gShopTileByRide.clear();

            // Ride entrances/exits (also covers shops' own queue-facing entrance where one exists).
            for (auto& ride : RideManager(gameState))
            {
                for (uint8_t stationIndex = 0; stationIndex < Limits::kMaxStationsPerRide; stationIndex++)
                {
                    if (stationIndex >= ride.numStations)
                        break;

                    const auto& station = ride.getStation(StationIndex::FromUnderlying(stationIndex));
                    if (!station.Entrance.IsNull())
                    {
                        TileCoordsXYZ goalLoc{ station.Entrance.x, station.Entrance.y, station.Entrance.z };
                        Direction facing = station.Entrance.direction;
                        TileCoordsXYZ approachLoc = goalLoc;
                        approachLoc += TileDirectionDelta[ApproachOffsetFor(facing)];
                        terminals.push_back({ approachLoc, facing, goalLoc });
                    }
                    if (!station.Exit.IsNull())
                    {
                        TileCoordsXYZ goalLoc{ station.Exit.x, station.Exit.y, station.Exit.z };
                        Direction facing = station.Exit.direction;
                        TileCoordsXYZ approachLoc = goalLoc;
                        approachLoc += TileDirectionDelta[ApproachOffsetFor(facing)];
                        terminals.push_back({ approachLoc, facing, goalLoc });
                    }
                }
            }

            // Park entrances: walkable from any facing direction (GuestPathfinding.cpp's
            // ENTRANCE_TYPE_PARK_ENTRANCE case does not check direction), so register all 4.
            for (auto& entrance : gameState.park.entrances)
            {
                TileCoordsXYZ goalLoc{ TileCoordsXYZ(CoordsXYZ{ entrance.x, entrance.y, entrance.z }) };
                for (Direction dir : kAllDirections)
                {
                    // `dir` offsets the approach tile away from the goal; walking back the other way
                    // (DirectionReverse) is the direction of travel that actually reaches the goal.
                    TileCoordsXYZ approachLoc = goalLoc;
                    approachLoc += TileDirectionDelta[dir];
                    terminals.push_back({ approachLoc, DirectionReverse(dir), goalLoc });
                }
            }

            // Peep spawns: guests stand directly on the path tile, so this is a forced path-tile node,
            // not a terminal.
            for (auto& spawn : gameState.peepSpawns)
            {
                TileCoordsXYZ loc{ TileCoordsXYZ(CoordsXYZ{ spawn.x, spawn.y, spawn.z }) };
                forcedNodeLocations.insert(loc);
            }

            // Shops: the shop's own TrackElement tile is the goal (GuestPathfinding.cpp:774-789), and
            // per that same logic it's walkable from any direction (no direction check there either).
            const auto mapSize = gameState.mapSize;
            for (int32_t y = 0; y < mapSize.y; y++)
            {
                for (int32_t x = 0; x < mapSize.x; x++)
                {
                    for (auto* el : TileElementsView<TrackElement>(TileCoordsXY{ x, y }))
                    {
                        if (el->isGhost())
                            continue;
                        RideId rideId = el->GetRideIndex();
                        if (rideId.IsNull())
                            continue;
                        Ride* ride = GetRide(rideId);
                        if (ride == nullptr || !ride->getRideTypeDescriptor().flags.has(RtdFlag::isShopOrFacility))
                            continue;

                        TileCoordsXYZ goalLoc{ x, y, el->baseHeight };
                        gShopTileByRide.emplace(rideId.ToUnderlying(), goalLoc);
                        for (Direction dir : kAllDirections)
                        {
                            TileCoordsXYZ approachLoc = goalLoc;
                            approachLoc += TileDirectionDelta[dir];
                            terminals.push_back({ approachLoc, DirectionReverse(dir), goalLoc });
                        }
                    }
                }
            }
        }

        std::vector<NavNodeId> ResolveGoalSeedNodes(
            const NavGoalId& goalId, const NavigationGraph& graph, const GameState_t& gameState)
        {
            std::vector<NavNodeId> seeds;
            switch (goalId.kind)
            {
                case NavGoalKind::rideStationEntrance:
                case NavGoalKind::rideStationExit:
                {
                    Ride* ride = GetRide(goalId.rideId);
                    if (ride == nullptr || goalId.stationIndex >= ride->numStations)
                        break;
                    const auto& station = ride->getStation(StationIndex::FromUnderlying(goalId.stationIndex));
                    const auto& tileCoords = (goalId.kind == NavGoalKind::rideStationEntrance) ? station.Entrance
                                                                                                : station.Exit;
                    if (tileCoords.IsNull())
                        break;
                    auto id = graph.FindNodeAt({ tileCoords.x, tileCoords.y, tileCoords.z });
                    if (!id.IsNull())
                        seeds.push_back(id);
                    break;
                }
                case NavGoalKind::shopEntrance:
                {
                    auto it = gShopTileByRide.find(goalId.rideId.ToUnderlying());
                    if (it == gShopTileByRide.end())
                        break;
                    auto id = graph.FindNodeAt(it->second);
                    if (!id.IsNull())
                        seeds.push_back(id);
                    break;
                }
                case NavGoalKind::parkEntranceAny:
                {
                    for (auto& entrance : gameState.park.entrances)
                    {
                        auto id = graph.FindNodeAt(TileCoordsXYZ(CoordsXYZ{ entrance.x, entrance.y, entrance.z }));
                        if (!id.IsNull())
                            seeds.push_back(id);
                    }
                    break;
                }
                case NavGoalKind::parkEntrance:
                {
                    if (goalId.index >= gameState.park.entrances.size())
                        break;
                    const auto& entrance = gameState.park.entrances[goalId.index];
                    auto id = graph.FindNodeAt(TileCoordsXYZ(CoordsXYZ{ entrance.x, entrance.y, entrance.z }));
                    if (!id.IsNull())
                        seeds.push_back(id);
                    break;
                }
                case NavGoalKind::peepSpawnAny:
                {
                    for (auto& spawn : gameState.peepSpawns)
                    {
                        auto id = graph.FindNodeAt(TileCoordsXYZ(CoordsXYZ{ spawn.x, spawn.y, spawn.z }));
                        if (!id.IsNull())
                            seeds.push_back(id);
                    }
                    break;
                }
            }
            return seeds;
        }

        // Reverse single/multi-source Dijkstra from `seeds` (the goal tile(s)) over the graph's
        // reverse adjacency, producing per-node (distance, nextHop) — nextHop is the direction to
        // step FROM that node to make progress toward the goal. See plan §4 for why guest tables
        // exclude banner-restricted edges and staff tables don't (that's the only per-peep-class
        // difference baked into the table; foreign-queue and patrol filtering happen at query time).
        void BuildDijkstra(
            const NavigationGraph& graph, const std::vector<NavNodeId>& seeds, bool excludeBannerRestricted,
            std::vector<NavGoalTableEntry>& out)
        {
            size_t n = graph.GetNodeCount();
            out.assign(n, NavGoalTableEntry{});

            std::vector<uint32_t> dist(n, NavGoalTableEntry::kInfiniteDistance);
            using QueueEntry = std::pair<uint32_t, uint32_t>; // (distance, NavNodeId::value)
            std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> pq;

            for (auto seed : seeds)
            {
                if (seed.IsNull() || seed.value >= n)
                    continue;
                if (dist[seed.value] != 0)
                {
                    dist[seed.value] = 0;
                    pq.push({ 0, seed.value });
                }
            }

            while (!pq.empty())
            {
                auto [d, nodeValue] = pq.top();
                pq.pop();
                if (d > dist[nodeValue])
                    continue;

                NavNodeId node{ nodeValue };
                for (const auto& edge : graph.GetReverseEdges(node))
                {
                    if (excludeBannerRestricted && edge.bannerRestricted)
                        continue;
                    if (edge.to.IsNull())
                        continue;

                    uint32_t newDist = d + edge.weight;
                    if (newDist < dist[edge.to.value])
                    {
                        dist[edge.to.value] = newDist;
                        out[edge.to.value].nextHop = edge.firstStepDirection;
                        pq.push({ newDist, edge.to.value });
                    }
                }
            }

            // Second pass: k-best alternatives, for the wandering/personality mechanism (plan §8).
            // A forward edge counts as "near-best" if taking it still gets within 150% of the true
            // optimum from this node — captures "plausible but suboptimal", not just the single best.
            for (size_t i = 0; i < n; i++)
            {
                out[i].distance = dist[i];
                if (dist[i] == NavGoalTableEntry::kInfiniteDistance)
                    continue;

                uint8_t mask = 0;
                uint32_t tolerance = dist[i] + dist[i] / 2;
                NavNodeId id{ static_cast<uint32_t>(i) };
                for (const auto& edge : graph.GetEdges(id))
                {
                    if (excludeBannerRestricted && edge.bannerRestricted)
                        continue;
                    if (edge.to.IsNull() || dist[edge.to.value] == NavGoalTableEntry::kInfiniteDistance)
                        continue;
                    uint32_t cost = edge.weight + dist[edge.to.value];
                    if (cost <= tolerance)
                        mask |= static_cast<uint8_t>(1u << edge.firstStepDirection);
                }
                out[i].kBestMask = mask;
            }
        }
    } // namespace

    const NavGoalTable* NavGoalTableCache::GetOrBuild(
        const NavGoalId& goalId, const NavigationGraph& graph, const GameState_t& gameState)
    {
        auto it = _tables.find(goalId);
        if (it != _tables.end() && it->second.graphVersion == graph.GetVersion())
            return &it->second;

        NavGoalTable table;
        table.goalId = goalId;
        table.graphVersion = graph.GetVersion();

        auto seeds = ResolveGoalSeedNodes(goalId, graph, gameState);
        bool excludeBannerRestricted = (goalId.peepClass == NavPeepClass::guest);
        BuildDijkstra(graph, seeds, excludeBannerRestricted, table.perNode);

        auto [insertedIt, _] = _tables.insert_or_assign(goalId, std::move(table));
        return &insertedIt->second;
    }

    void NavGoalTableCache::InvalidateAll()
    {
        _tables.clear();
    }

    void RefreshGoalNodesAndGraph(NavigationGraph& graph, const GameState_t& gameState)
    {
        std::unordered_set<TileCoordsXYZ, TileCoordsXYZHash> forcedNodeLocations;
        std::vector<NavGoalTerminal> terminals;
        CollectGoals(gameState, forcedNodeLocations, terminals);

        graph.SetForcedNodeLocations(std::move(forcedNodeLocations));
        graph.SetGoalTerminals(std::move(terminals));
        graph.RebuildDirtyRegions();
    }

    NavGoalTableCache& GetGoalTableCache()
    {
        static NavGoalTableCache cache;
        return cache;
    }

} // namespace OpenRCT2::Navigation
