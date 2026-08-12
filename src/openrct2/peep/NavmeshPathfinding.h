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
#include "../world/Location.hpp"
#include "../world/NavigationGraphGoals.h"

namespace OpenRCT2
{
    struct Peep;
}

namespace OpenRCT2::PathFinding
{
    // Read-only O(1) replacement for PeepPathfindHeuristicSearch's DFS: looks up the precomputed
    // Dijkstra table for `goalId` and returns the next-hop direction from `loc`, or kInvalidDirection
    // if unreachable (mirrors ChooseDirection's "no direction chosen" case) or if `loc` isn't a
    // registered graph node.
    //
    // Phase A scope: does NOT yet read/write Peep::PathfindHistory (the wandering mechanism, plan
    // §8) or fall back to a bounded local search on a foreign-queue/patrol-area conflict (plan §9).
    // Both are deferred to Phase B, when a call site's return value actually starts driving real
    // peep movement and those correctness details become player-visible. For now this only needs to
    // be good enough for shadow-mode divergence comparison against the existing algorithm.
    Direction NavmeshChooseDirection(const TileCoordsXYZ& loc, const Navigation::NavGoalId& goalId);

    // Computes goalId's table lookup and compares it against `oldResult` (the real ChooseDirection()
    // result already used for movement this call, unaffected by this function). Logs a divergence via
    // LOG_INFO when they differ. No-op unless IsShadowModeEnabled(). Never mutates `peep`.
    void ShadowCompareChooseDirection(
        const Peep& peep, const TileCoordsXYZ& loc, const Navigation::NavGoalId& goalId, Direction oldResult);

    bool IsShadowModeEnabled();
    void SetShadowModeEnabled(bool enabled);

} // namespace OpenRCT2::PathFinding
