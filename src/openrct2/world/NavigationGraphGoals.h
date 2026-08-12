/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../Identifiers.h"
#include "Location.hpp"
#include "NavigationGraph.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace OpenRCT2
{
    struct GameState_t;
}

namespace OpenRCT2::Navigation
{
    // Guests respect queue-line banners; staff (per PathGetPermittedEdges(ignoreBanners=true, ...))
    // don't. This is the only axis that changes which edges are traversable, so it's the only thing
    // that multiplies the number of cached tables per goal (see NavGoalTable's doc comment).
    enum class NavPeepClass : uint8_t
    {
        guest,
        staff,
    };

    enum class NavGoalKind : uint8_t
    {
        rideStationEntrance,
        rideStationExit,
        shopEntrance,
        parkEntranceAny, // multi-source: nearest of every current park entrance
        parkEntrance,    // one specific park entrance, by index into ParkData::entrances (sticky goal)
        peepSpawnAny,    // multi-source: nearest of every current peep spawn
    };

    struct NavGoalId
    {
        NavGoalKind kind;
        NavPeepClass peepClass = NavPeepClass::guest;
        RideId rideId = RideId::GetNull();
        uint8_t stationIndex = 0;
        uint16_t index = 0;

        constexpr bool operator==(const NavGoalId&) const = default;
    };

    struct NavGoalIdHash
    {
        size_t operator()(const NavGoalId& id) const noexcept
        {
            size_t h = static_cast<size_t>(id.kind);
            h = h * 31 + static_cast<size_t>(id.peepClass);
            h = h * 31 + static_cast<size_t>(id.rideId.ToUnderlying());
            h = h * 31 + static_cast<size_t>(id.stationIndex);
            h = h * 31 + static_cast<size_t>(id.index);
            return h;
        }
    };

    struct NavGoalTableEntry
    {
        static constexpr uint32_t kInfiniteDistance = 0xFFFFFFFFu;

        uint32_t distance = kInfiniteDistance;
        Direction nextHop = kInvalidDirection;
        uint8_t kBestMask = 0; // bit i set: forward edge in direction i is a near-optimal alternative
    };

    // One table per (goal, peep-class) — NOT per-peep, NOT per-queue-context, NOT per-patrol-area.
    // Foreign-queue and staff-patrol exceptions are handled at query time by NavmeshPathfinding
    // falling back to a small bounded local search, rather than by building more table variants —
    // see the implementation plan (.claude/plans/magical-baking-finch.md, §4) for why.
    struct NavGoalTable
    {
        NavGoalId goalId;
        std::vector<NavGoalTableEntry> perNode; // indexed by NavNodeId::value
        uint32_t graphVersion = 0;
    };

    class NavGoalTableCache
    {
    public:
        // Builds (or returns the cached, still-valid) table for this goal. `graph` must already
        // reflect the current map state (i.e. RefreshGoalNodesAndGraph was called this tick).
        const NavGoalTable* GetOrBuild(const NavGoalId& goalId, const NavigationGraph& graph, const GameState_t& gameState);

        void InvalidateAll();

    private:
        std::unordered_map<NavGoalId, NavGoalTable, NavGoalIdHash> _tables;
    };

    // Rebuilds the graph's forced-node/goal-terminal sets from the current ride entrances/exits,
    // shop tiles, park entrances, and peep spawns, then rebuilds any dirty graph regions. Call once
    // per tick, before any peep pathfinding decision (mirrors gameStateUpdateLogic's existing
    // MapUpdatePathWideFlags -> PeepUpdateAll ordering — see plan §5).
    void RefreshGoalNodesAndGraph(NavigationGraph& graph, const GameState_t& gameState);

    NavGoalTableCache& GetGoalTableCache();

} // namespace OpenRCT2::Navigation
