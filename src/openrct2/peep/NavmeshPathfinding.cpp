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
#include "../world/NavigationGraph.h"

namespace OpenRCT2::PathFinding
{
    namespace
    {
        bool gShadowModeEnabled = false;
    }

    Direction NavmeshChooseDirection(const TileCoordsXYZ& loc, const Navigation::NavGoalId& goalId)
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

    void ShadowCompareChooseDirection(
        const Peep& peep, const TileCoordsXYZ& loc, const Navigation::NavGoalId& goalId, Direction oldResult)
    {
        if (!gShadowModeEnabled)
            return;

        Direction newResult = NavmeshChooseDirection(loc, goalId);
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

} // namespace OpenRCT2::PathFinding
