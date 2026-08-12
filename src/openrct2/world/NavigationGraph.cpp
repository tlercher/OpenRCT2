/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "NavigationGraph.h"

#include "../GameState.h"
#include "../core/Numerics.hpp"
#include "../peep/GuestPathfinding.h"
#include "Footpath.h"
#include "Map.h"
#include "TileElementsView.h"
#include "tile_element/PathElement.h"

#include <algorithm>
#include <bit>
#include <optional>

namespace OpenRCT2::Navigation
{
    namespace
    {
        constexpr uint32_t kRegionSize = 64; // matches PatrolArea's cell size

        constexpr uint32_t RegionIdFor(const TileCoordsXY& loc, uint32_t regionCols)
        {
            return static_cast<uint32_t>(loc.y / kRegionSize) * regionCols + static_cast<uint32_t>(loc.x / kRegionSize);
        }

        const PathElement* FindFirstPathElementAt(const TileCoordsXYZ& loc)
        {
            for (auto* el : TileElementsView<PathElement>(TileCoordsXY{ loc.x, loc.y }))
            {
                if (el->isGhost())
                    continue;
                if (el->baseHeight != loc.z)
                    continue;
                return el;
            }
            return nullptr;
        }

        // Structural classification only (junction/dead-end vs. an ordinary 2-edge "thin" corridor
        // tile that collapses into an edge chain). This is a topology question, distinct from
        // PathFinding::PathIsThinJunction, which is a *behavioral* loop/junction-budget test applied
        // during search on top of this topology (reused later by NavmeshPathfinding, not here).
        std::optional<NavNodeKind> ClassifyStructural(const PathElement* el)
        {
            if (el->isWide())
                return NavNodeKind::wideEntry;

            uint8_t numEdges = static_cast<uint8_t>(std::popcount(static_cast<uint32_t>(el->getEdges())));
            if (numEdges < 2)
                return NavNodeKind::deadEnd;
            if (numEdges > 2)
                return NavNodeKind::junction;
            return std::nullopt; // corridor tile
        }

        struct StepResult
        {
            TileCoordsXYZ loc; // resolved location, z snapped to the found element's baseHeight
            const PathElement* element = nullptr;
            bool isWide = false;
            bool isQueue = false;
            RideId queueRideId = RideId::GetNull();
        };

        // Mirrors OpenRCT2::PathFinding::FootpathElementNextInDirection (GuestPathfinding.cpp), but
        // returns the resolved element/location instead of a coarse classification, since the graph
        // builder needs to keep walking the actual tile chain rather than stop at the first result.
        // `terminals` lets a step land on a non-path goal tile (ride entrance/exit, shop, park
        // entrance) registered via NavigationGraph::SetGoalTerminals — see NavGoalTerminal's doc
        // comment for why those can't just be found via TileElementsView<PathElement>.
        std::optional<StepResult> StepPath(
            TileCoordsXYZ loc, Direction direction,
            const std::unordered_map<TileCoordsXYZ, std::vector<std::pair<Direction, TileCoordsXYZ>>, TileCoordsXYZHash>&
                terminals)
        {
            if (auto it = terminals.find(loc); it != terminals.end())
            {
                for (auto& [approachDirection, goalLoc] : it->second)
                {
                    if (approachDirection == direction)
                    {
                        StepResult result;
                        result.loc = goalLoc;
                        result.element = nullptr;
                        return result;
                    }
                }
            }

            const PathElement* fromElement = FindFirstPathElementAt(loc);
            if (fromElement != nullptr && fromElement->isSloped() && fromElement->getSlopeDirection() == direction)
            {
                loc.z += 2;
            }

            loc += TileDirectionDelta[direction];

            for (auto* el : TileElementsView<PathElement>(TileCoordsXY{ loc.x, loc.y }))
            {
                if (el->isGhost())
                    continue;
                if (!FootpathIsZAndDirectionValid(*el, loc.z, direction))
                    continue;

                StepResult result;
                result.loc = { loc.x, loc.y, el->baseHeight };
                result.element = el;
                result.isWide = el->isWide();
                result.isQueue = el->isQueue() && !el->getRideIndex().IsNull();
                result.queueRideId = el->isQueue() ? el->getRideIndex() : RideId::GetNull();
                return result;
            }
            return std::nullopt;
        }
    } // namespace

    void NavigationGraph::SetForcedNodeLocations(std::unordered_set<TileCoordsXYZ, TileCoordsXYZHash> locations)
    {
        _forcedNodeLocations = std::move(locations);
    }

    void NavigationGraph::SetGoalTerminals(std::vector<NavGoalTerminal> terminals)
    {
        _goalTerminalsByApproach.clear();
        for (auto& terminal : terminals)
        {
            _goalTerminalsByApproach[terminal.approachLoc].emplace_back(terminal.approachDirection, terminal.goalLoc);
        }
    }

    void NavigationGraph::MarkRegionDirty(const TileCoordsXY& tileLoc)
    {
        _anyDirty = true;
        if (_regionCols == 0 || _regionRows == 0)
            return; // graph not built yet; RebuildAll() will pick up current map state regardless

        auto id = RegionIdFor(tileLoc, _regionCols);
        if (id < _regions.size())
            _regions[id].dirty = true;
    }

    void NavigationGraph::MarkAllDirty()
    {
        _anyDirty = true;
        for (auto& region : _regions)
            region.dirty = true;
    }

    void NavigationGraph::RebuildDirtyRegions()
    {
        // Phase A: any dirty region triggers a full rebuild — see NavRegion's doc comment for why
        // this is an acceptable simplification for now (map edits are rare relative to the per-tick,
        // per-peep decision volume this feature targets).
        if (!_anyDirty)
            return;
        RebuildFromScratch();
    }

    void NavigationGraph::RebuildAll()
    {
        RebuildFromScratch();
    }

    NavNodeId NavigationGraph::FindNodeAt(const TileCoordsXYZ& loc) const
    {
        auto it = _nodeByLocation.find(loc);
        if (it == _nodeByLocation.end())
            return kInvalidNavNodeId;
        return it->second;
    }

    std::span<const NavEdge> NavigationGraph::GetEdges(NavNodeId id) const
    {
        if (id.IsNull() || id.value >= _nodes.size())
            return {};
        const auto& node = _nodes[id.value];
        return std::span<const NavEdge>(_edges).subspan(node.edgeOffset, node.edgeCount);
    }

    std::span<const NavEdge> NavigationGraph::GetReverseEdges(NavNodeId id) const
    {
        if (id.IsNull() || id.value >= _nodes.size() || id.value >= _reverseNodeOffset.size())
            return {};
        return std::span<const NavEdge>(_reverseEdges).subspan(_reverseNodeOffset[id.value], _reverseNodeCount[id.value]);
    }

    void NavigationGraph::RebuildFromScratch()
    {
        _nodes.clear();
        _edges.clear();
        _reverseEdges.clear();
        _nodeByLocation.clear();
        _regions.clear();

        const auto& gameState = getGameState();
        const auto mapSize = gameState.mapSize;
        if (mapSize.x <= 0 || mapSize.y <= 0)
        {
            _regionCols = 0;
            _regionRows = 0;
            _anyDirty = false;
            return;
        }

        _regionCols = (static_cast<uint32_t>(mapSize.x) + kRegionSize - 1) / kRegionSize;
        _regionRows = (static_cast<uint32_t>(mapSize.y) + kRegionSize - 1) / kRegionSize;
        _regions.assign(static_cast<size_t>(_regionCols) * _regionRows, NavRegion{});

        auto registerNode = [&](const TileCoordsXYZ& loc, NavNodeKind kind) {
            if (_nodeByLocation.contains(loc))
                return; // already registered (e.g. overlaid path elements at the same tile)

            NavNode node;
            node.location = loc;
            node.kind = kind;
            node.regionId = static_cast<uint16_t>(RegionIdFor(TileCoordsXY{ loc.x, loc.y }, _regionCols));

            NavNodeId id{ static_cast<uint32_t>(_nodes.size()) };
            _nodes.push_back(node);
            _nodeByLocation.emplace(loc, id);
            if (node.regionId < _regions.size())
                _regions[node.regionId].nodeIds.push_back(id);
        };

        // Pass 1: register every structural node (junction / dead-end / wide-entry), every forced
        // path-tile goal (e.g. a peep spawn standing on a path), and every non-path goal terminal
        // (ride entrance/exit, shop, park entrance — see NavGoalTerminal), so pass 2 can terminate
        // chain walks by a simple nodeByLocation lookup.
        for (int32_t y = 0; y < mapSize.y; y++)
        {
            for (int32_t x = 0; x < mapSize.x; x++)
            {
                for (auto* el : TileElementsView<PathElement>(TileCoordsXY{ x, y }))
                {
                    if (el->isGhost())
                        continue;

                    TileCoordsXYZ loc{ x, y, el->baseHeight };
                    auto kind = ClassifyStructural(el);
                    bool isForcedGoal = _forcedNodeLocations.contains(loc);
                    if (!kind.has_value() && !isForcedGoal)
                        continue; // ordinary corridor tile, not a node

                    registerNode(loc, kind.value_or(NavNodeKind::goal));
                }
            }
        }

        for (auto& [approachLoc, entries] : _goalTerminalsByApproach)
        {
            (void)approachLoc;
            for (auto& entry : entries)
                registerNode(entry.second, NavNodeKind::goal);
        }

        // Pass 2: for every node that isn't a wide-entry (wide areas hand off to local search rather
        // than being routed through per-tile, see NavigationGraph.h), walk each raw edge direction as
        // a chain of corridor tiles until hitting another registered node, and emit one NavEdge.
        struct PendingEdge
        {
            NavNodeId from;
            NavEdge edge;
        };
        std::vector<PendingEdge> pending;

        for (size_t i = 0; i < _nodes.size(); i++)
        {
            const NavNode& node = _nodes[i];
            if (node.kind == NavNodeKind::wideEntry)
                continue;

            const PathElement* startElement = FindFirstPathElementAt(node.location);
            if (startElement == nullptr)
                continue; // goal-terminal node (ride entrance/exit, shop, park entrance): a terminal
                          // endpoint has no outgoing edges of its own, it's only ever a `to`, never a `from`.

            uint32_t rawEdges = startElement->getEdges() & 0x0F;
            for (Direction dir : kAllDirections)
            {
                if (!(rawEdges & (1u << dir)))
                    continue;

                TileCoordsXYZ loc = node.location;
                Direction direction = dir;
                uint16_t weight = 0;
                bool sawQueue = false;
                RideId queueRideId = RideId::GetNull();
                bool bannerRestricted = !(PathFinding::PathGetPermittedEdges(false, startElement) & (1 << dir));

                bool reachedNode = false;
                NavNodeId toId = kInvalidNavNodeId;
                Direction lastStepDirection = direction;

                // Bound the walk defensively; a real corridor chain is at most map-diagonal long.
                const int32_t guardLimit = mapSize.x + mapSize.y + 4;
                for (int32_t guard = 0; guard < guardLimit; guard++)
                {
                    auto step = StepPath(loc, direction, _goalTerminalsByApproach);
                    if (!step.has_value())
                        break; // ran off the map / into nothing; discard this edge

                    weight++;
                    if (step->isQueue)
                    {
                        sawQueue = true;
                        queueRideId = step->queueRideId;
                    }

                    if (step->isWide)
                    {
                        auto it = _nodeByLocation.find(step->loc);
                        if (it != _nodeByLocation.end())
                        {
                            reachedNode = true;
                            toId = it->second;
                        }
                        break; // wide tiles always terminate the chain
                    }

                    auto it = _nodeByLocation.find(step->loc);
                    if (it != _nodeByLocation.end())
                    {
                        reachedNode = true;
                        toId = it->second;
                        break;
                    }

                    if (step->element == nullptr)
                        break; // defensive: a terminal hit that wasn't pre-registered as a node; discard edge

                    // Corridor tile: continue in whichever of its (exactly 2) raw edges isn't the one
                    // we just arrived from.
                    uint32_t edges = step->element->getEdges() & 0x0F;
                    Direction cameFrom = DirectionReverse(direction);
                    edges &= ~(1u << cameFrom);
                    if (!(PathFinding::PathGetPermittedEdges(false, step->element) & static_cast<int32_t>(edges)))
                    {
                        bannerRestricted = true;
                    }
                    int32_t nextDir = Numerics::bitScanForward(edges);
                    if (nextDir == -1)
                        break; // malformed (a "corridor" tile without a continuation); discard edge

                    loc = step->loc;
                    direction = static_cast<Direction>(nextDir);
                    lastStepDirection = direction;
                }

                if (!reachedNode)
                    continue;

                NavEdge edge;
                edge.to = toId;
                edge.firstStepDirection = dir;
                edge.weight = weight;
                edge.queueRideId = sawQueue ? queueRideId : RideId::GetNull();
                edge.isQueue = sawQueue;
                edge.bannerRestricted = bannerRestricted;
                (void)lastStepDirection;

                pending.push_back(PendingEdge{ NavNodeId{ static_cast<uint32_t>(i) }, edge });
            }
        }

        // Build forward CSR: sort pending edges by source node so each node's edges are contiguous.
        std::stable_sort(pending.begin(), pending.end(), [](const PendingEdge& a, const PendingEdge& b) {
            return a.from.value < b.from.value;
        });

        _edges.reserve(pending.size());
        for (size_t i = 0; i < _nodes.size(); i++)
        {
            _nodes[i].edgeOffset = static_cast<uint32_t>(_edges.size());
            _nodes[i].edgeCount = 0;
        }
        for (auto& p : pending)
        {
            if (_nodes[p.from.value].edgeCount == 0)
                _nodes[p.from.value].edgeOffset = static_cast<uint32_t>(_edges.size());
            _nodes[p.from.value].edgeCount++;
            _edges.push_back(p.edge);
        }

        // Build reverse CSR (same layout, indexed by destination node) for the reverse-Dijkstra pass
        // goal tables run in NavigationGraphGoals.
        std::vector<uint32_t> reverseCount(_nodes.size(), 0);
        for (auto& p : pending)
            reverseCount[p.edge.to.value]++;

        std::vector<uint32_t> reverseOffset(_nodes.size(), 0);
        uint32_t running = 0;
        for (size_t i = 0; i < _nodes.size(); i++)
        {
            reverseOffset[i] = running;
            running += reverseCount[i];
        }
        _reverseEdges.assign(pending.size(), NavEdge{});
        std::vector<uint32_t> cursor = reverseOffset;
        for (auto& p : pending)
        {
            NavEdge reverseEdge = p.edge;
            reverseEdge.to = p.from; // reverse adjacency points back to the original source
            _reverseEdges[cursor[p.edge.to.value]++] = reverseEdge;
        }
        _reverseNodeOffset = std::move(reverseOffset);
        _reverseNodeCount = std::move(reverseCount);

        _graphVersion++;
        _anyDirty = false;
        for (auto& region : _regions)
            region.dirty = false;
    }

    NavigationGraph& GetNavigationGraph()
    {
        // Placeholder singleton for Phase A (graph infra + shadow mode). Wiring this into GameState_t
        // (so it resets correctly on park load / swapGameState for multiplayer) is tracked separately.
        static NavigationGraph graph;
        return graph;
    }

} // namespace OpenRCT2::Navigation
