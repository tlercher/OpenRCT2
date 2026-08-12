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

#include <cstdint>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace OpenRCT2::Navigation
{
    // A dense index into NavigationGraph::nodes. Deliberately NOT a pointer: the graph is stored
    // as flat CSR arrays specifically to avoid the scattered-pointer-chasing pattern that made the
    // old per-peep tile-by-tile search the single largest L1D-cache-miss contributor in the game
    // (see GuestPathfinding.cpp's PeepPathfindHeuristicSearch / MapGetFirstElementAt chain).
    struct NavNodeId
    {
        static constexpr uint32_t kInvalidValue = 0xFFFFFFFFu;

        uint32_t value = kInvalidValue;

        constexpr bool IsNull() const
        {
            return value == kInvalidValue;
        }

        constexpr bool operator==(const NavNodeId&) const = default;
    };

    inline constexpr NavNodeId kInvalidNavNodeId{};

    enum class NavNodeKind : uint8_t
    {
        junction, // >2 raw path edges
        deadEnd,  // <2 raw path edges
        wideEntry, // a wide-flagged path tile; peers hand off to local search once here (see NavmeshPathfinding)
        goal,     // forced node for a ride entrance/exit, shop, park entrance or peep spawn that would
                  // otherwise have collapsed into a 2-edge corridor chain
    };

    struct NavEdge
    {
        NavNodeId to;
        Direction firstStepDirection = kInvalidDirection; // direction leaving the source node onto this chain
        uint16_t weight = 0;                              // tile count of the collapsed chain
        RideId queueRideId = RideId::GetNull();            // null if the chain contains no queue tiles
        bool isQueue = false;
        bool bannerRestricted = false; // true if a banner blocks this direction for guests (not staff)
                                        // anywhere along the collapsed chain; see PathGetPermittedEdges.
    };

    struct NavNode
    {
        TileCoordsXYZ location;
        NavNodeKind kind = NavNodeKind::junction;
        uint8_t edgeCount = 0;
        uint32_t edgeOffset = 0; // CSR offset into NavigationGraph::_edges / _reverseEdges
        uint16_t regionId = 0;
    };

    // 64x64-tile chunk, matching PatrolArea's cell size. Phase A intentionally does not implement
    // true per-region incremental rebuild (the trickiest part is edges whose collapsed chain crosses
    // a region boundary); it tracks dirty regions so a future optimization can scope rebuilds to just
    // the affected area, but for now any dirty region triggers a full RebuildAll(). This is deliberate:
    // map edits are rare compared to the per-tick, per-peep decision volume this feature targets, so
    // even an O(map size) rebuild on edit is dominated by the win of eliminating the old 15,000-tile
    // search that used to run on every junction decision.
    struct NavRegion
    {
        bool dirty = true;
        std::vector<NavNodeId> nodeIds;
    };

    struct TileCoordsXYZHash
    {
        size_t operator()(const TileCoordsXYZ& loc) const noexcept
        {
            // Simple mix; coordinates are small (map size is capped well under 16 bits per axis).
            size_t h = static_cast<size_t>(static_cast<uint32_t>(loc.x));
            h = h * 31 + static_cast<size_t>(static_cast<uint32_t>(loc.y));
            h = h * 31 + static_cast<size_t>(static_cast<uint32_t>(loc.z));
            return h;
        }
    };

    // A goal tile that is NOT itself a footpath tile (a ride entrance/exit, a shop's track tile, a
    // park entrance) but is reached by walking off an adjacent path tile in a specific direction —
    // mirrors how PeepPathfindHeuristicSearch treats TileElementType::entrance/track neighbours as
    // search-ending "found" results (GuestPathfinding.cpp:774-836), rather than as ordinary path
    // tiles. NavigationGraphGoals computes these (it owns ride/park domain knowledge); NavigationGraph
    // just needs to know "standing at approachLoc facing approachDirection reaches goalLoc" so the
    // chain walker can terminate there like any other node.
    struct NavGoalTerminal
    {
        TileCoordsXYZ approachLoc;
        Direction approachDirection = kInvalidDirection;
        TileCoordsXYZ goalLoc;
    };

    // Goal-agnostic navmesh graph over the footpath network. Nodes are junctions, dead ends, wide-path
    // entries, and (externally supplied) goal tiles; chains of ordinary 2-edge "thin" path tiles between
    // nodes are collapsed into single weighted edges. This class knows nothing about peeps, rides, or
    // goals — goal enumeration and shortest-path tables live in NavigationGraphGoals.
    class NavigationGraph
    {
    public:
        // Goal tiles that ARE footpath tiles (e.g. a peep spawn standing on a path) must still be
        // forced to become nodes even when they'd otherwise sit on a 2-edge "thin" chain — a Dijkstra
        // table needs a node to seed/target. NavigationGraphGoals populates this before (re)building.
        void SetForcedNodeLocations(std::unordered_set<TileCoordsXYZ, TileCoordsXYZHash> locations);

        // Goal tiles that are NOT footpath tiles (ride entrances/exits, shops, park entrances) — see
        // NavGoalTerminal. NavigationGraphGoals populates this before (re)building.
        void SetGoalTerminals(std::vector<NavGoalTerminal> terminals);

        void MarkRegionDirty(const TileCoordsXY& tileLoc);
        void MarkAllDirty();

        // Phase A: rebuilds everything if anything is dirty (see NavRegion comment above).
        void RebuildDirtyRegions();
        void RebuildAll();

        NavNodeId FindNodeAt(const TileCoordsXYZ& loc) const;

        const NavNode& GetNode(NavNodeId id) const
        {
            return _nodes[id.value];
        }

        std::span<const NavEdge> GetEdges(NavNodeId id) const;
        std::span<const NavEdge> GetReverseEdges(NavNodeId id) const;

        size_t GetNodeCount() const
        {
            return _nodes.size();
        }

        // Monotonic counter bumped every time the graph topology actually changes; goal tables cache
        // this to know when they've gone stale.
        uint32_t GetVersion() const
        {
            return _graphVersion;
        }

    private:
        std::vector<NavNode> _nodes;
        std::vector<NavEdge> _edges;
        std::vector<NavEdge> _reverseEdges;
        std::vector<uint32_t> _reverseNodeOffset; // parallel to _nodes; reverse CSR slice start
        std::vector<uint32_t> _reverseNodeCount;  // parallel to _nodes; reverse CSR slice length
        std::unordered_map<TileCoordsXYZ, NavNodeId, TileCoordsXYZHash> _nodeByLocation;
        std::vector<NavRegion> _regions;
        std::unordered_set<TileCoordsXYZ, TileCoordsXYZHash> _forcedNodeLocations;
        // approachLoc -> list of (direction, goalLoc), almost always 0-1 entries per tile in practice.
        std::unordered_map<TileCoordsXYZ, std::vector<std::pair<Direction, TileCoordsXYZ>>, TileCoordsXYZHash>
            _goalTerminalsByApproach;
        uint32_t _regionCols = 0;
        uint32_t _regionRows = 0;
        uint32_t _graphVersion = 0;
        bool _anyDirty = true;

        void RebuildFromScratch();
    };

    NavigationGraph& GetNavigationGraph();

} // namespace OpenRCT2::Navigation
