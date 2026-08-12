/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "NavmeshPathfinding.h"

#include "../Diagnostic.h"
#include "../GameState.h"
#include "../entity/Peep.h"
#include "../entity/Staff.h"
#include "../world/NavigationGraph.h"
#include "../world/TileElementsView.h"
#include "../world/tile_element/PathElement.h"
#include "GuestPathfinding.h"

#include <algorithm>
#include <bit>

namespace OpenRCT2::PathFinding
{
    namespace
    {
        bool gShadowModeEnabled = false;
        bool gLiveModeEnabled = true;

        // Bounded local-search budget for the fallback path (plan §9): only needs to resolve an
        // immediate local ambiguity (a foreign queue or patrol boundary right ahead), the
        // precomputed table already guarantees a route exists beyond it. Tiny next to the old
        // algorithm's normal 15,000/50,000-tile budget.
        constexpr int32_t kFallbackTileBudget = 400;

        PathElement* FindFirstPathElementAt(const TileCoordsXYZ& loc)
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

        const Navigation::NavEdge* FindForwardEdge(
            const Navigation::NavigationGraph& graph, Navigation::NavNodeId node, Direction direction)
        {
            for (const auto& edge : graph.GetEdges(node))
            {
                if (edge.firstStepDirection == direction)
                    return &edge;
            }
            return nullptr;
        }
    } // namespace

    Direction NavmeshTableLookup(const TileCoordsXYZ& loc, const Navigation::NavGoalId& goalId)
    {
        auto& graph = Navigation::GetNavigationGraph();
        auto node = graph.FindNodeAt(loc);
        if (node.IsNull())
            return kInvalidDirection;

        auto* table = Navigation::GetGoalTableCache().GetOrBuild(goalId, graph, getGameState());
        if (table == nullptr || node.value >= table->perNode.size())
            return kInvalidDirection;

        const auto& entry = table->perNode[node.value];
        if (entry.distance == Navigation::NavGoalTableEntry::kInfiniteDistance)
            return kInvalidDirection;

        return entry.nextHop;
    }

    Direction NavmeshChooseDirection(
        const TileCoordsXYZ& loc, const Navigation::NavGoalId& goalId, const TileCoordsXYZ& rawGoal, Peep& peep,
        bool ignoreForeignQueues, RideId queueRideIndex)
    {
        auto& graph = Navigation::GetNavigationGraph();
        auto node = graph.FindNodeAt(loc);
        if (node.IsNull())
        {
            // Shouldn't normally happen (real decisions only occur at junctions, which are always
            // graph nodes) - defensively fall back to the old algorithm at full budget rather than
            // give up, e.g. in case of an edit-race with the once-per-tick graph rebuild.
            return ChooseDirection(loc, rawGoal, peep, ignoreForeignQueues, queueRideIndex);
        }

        auto* table = Navigation::GetGoalTableCache().GetOrBuild(goalId, graph, getGameState());
        if (table == nullptr || node.value >= table->perNode.size())
            return kInvalidDirection;

        const auto& entry = table->perNode[node.value];
        if (entry.distance == Navigation::NavGoalTableEntry::kInfiniteDistance)
            return kInvalidDirection;

        // --- Wandering (plan §8): reproduce the old algorithm's "confused guest" behaviour without
        // re-running a search. `isThin`/`permittedEdges` mirror ChooseDirection's own tile-element
        // handling (GuestPathfinding.cpp:1264-1289) using the same exposed helpers, single-element
        // simplification (matches that function's own documented behaviour for overlaid paths).
        PathElement* pathElement = FindFirstPathElementAt(loc);
        bool isThin = pathElement != nullptr && PathIsThinJunction(pathElement, loc);
        uint8_t permittedEdges = pathElement != nullptr
            ? static_cast<uint8_t>(PathGetPermittedEdges(peep.is<Staff>(), pathElement) & 0x0F)
            : 0;

        // New goal for this peep: reset history exactly as ChooseDirection does (GuestPathfinding.cpp
        // ~1351-1364).
        if (!DirectionValid(peep.PathfindGoal.direction) || peep.PathfindGoal != rawGoal)
        {
            peep.PathfindGoal = { rawGoal, 0 };
            TileCoordsXYZD nullPos;
            nullPos.SetNull();
            std::fill(std::begin(peep.PathfindHistory), std::end(peep.PathfindHistory), nullPos);
        }

        Direction chosenDirection = entry.nextHop;
        int historySlot = -1;
        if (isThin)
        {
            for (std::size_t i = 0; i < peep.PathfindHistory.size(); ++i)
            {
                if (peep.PathfindHistory[i] == loc)
                {
                    historySlot = static_cast<int>(i);
                    uint8_t untried = static_cast<uint8_t>(peep.PathfindHistory[i].direction) & permittedEdges;
                    uint8_t candidates = untried & entry.kBestMask;
                    if (candidates != 0)
                    {
                        chosenDirection = static_cast<Direction>(std::countr_zero(static_cast<unsigned>(candidates)));
                    }
                    // else: everything untried-and-near-optimal has been tried; fall through to the
                    // true optimum (entry.nextHop), same as the old algorithm eventually converging.
                    break;
                }
            }
        }

        // --- Foreign-queue / patrol fallback (plan §4, §9): the shared table doesn't encode these
        // per-peep exceptions, so validate the chosen edge at query time and drop to a small bounded
        // local search if it's not actually usable right now.
        const auto* edge = FindForwardEdge(graph, node, chosenDirection);
        bool needsFallback = false;
        if (edge != nullptr)
        {
            if (ignoreForeignQueues && edge->isQueue && !edge->queueRideId.IsNull() && edge->queueRideId != queueRideIndex)
                needsFallback = true;
        }
        if (auto* staff = peep.as<Staff>(); staff != nullptr && staff->isMechanic())
        {
            TileCoordsXYZ nextTile = loc;
            nextTile += TileDirectionDelta[chosenDirection];
            if (!staff->isLocationInPatrol(nextTile.ToCoordsXY()))
                needsFallback = true;
        }

        if (needsFallback)
        {
            return ChooseDirection(loc, rawGoal, peep, ignoreForeignQueues, queueRideIndex, kFallbackTileBudget);
        }

        // --- Bookkeeping (plan §8): ported unchanged from ChooseDirection (GuestPathfinding.cpp
        // ~1521-1556) - doesn't depend on how chosenDirection was derived, only which direction and
        // which junction.
        if (isThin)
        {
            if (historySlot >= 0)
            {
                peep.PathfindHistory[historySlot].direction &= ~(1 << chosenDirection);
                peep.PathfindHistory[historySlot].direction &= ~(1 << DirectionReverse(peep.PeepDirection));
            }
            else
            {
                int32_t i = peep.PathfindGoal.direction++;
                peep.PathfindGoal.direction &= 3;
                peep.PathfindHistory[i] = { loc, permittedEdges };
                peep.PathfindHistory[i].direction &= ~(1 << chosenDirection);
                peep.PathfindHistory[i].direction &= ~(1 << DirectionReverse(peep.PeepDirection));
            }
        }

        return chosenDirection;
    }

    void ShadowCompareChooseDirection(
        const Peep& peep, const TileCoordsXYZ& loc, const Navigation::NavGoalId& goalId, Direction oldResult)
    {
        if (!gShadowModeEnabled)
            return;

        Direction newResult = NavmeshTableLookup(loc, goalId);
        if (newResult == oldResult)
            return;

        LOG_INFO(
            "[navmesh-shadow] peep=%u (%s) loc=%d,%d,%d goalKind=%u rideId=%u station=%u old=%d new=%d",
            peep.id.ToUnderlying(), peep.GetName().c_str(), loc.x, loc.y, loc.z, static_cast<unsigned>(goalId.kind),
            goalId.rideId.ToUnderlying(), goalId.stationIndex, oldResult, newResult);
    }

    bool IsShadowModeEnabled()
    {
        return gShadowModeEnabled;
    }

    void SetShadowModeEnabled(bool enabled)
    {
        gShadowModeEnabled = enabled;
    }

    bool IsLiveModeEnabled()
    {
        return gLiveModeEnabled;
    }

    void SetLiveModeEnabled(bool enabled)
    {
        gLiveModeEnabled = enabled;
    }

} // namespace OpenRCT2::PathFinding
