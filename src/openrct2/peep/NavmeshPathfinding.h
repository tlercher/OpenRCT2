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
    // Pure O(1) table lookup: the precomputed Dijkstra table's next-hop direction from `loc` toward
    // `goalId`, or kInvalidDirection if unreachable or `loc` isn't a registered graph node. No peep
    // state read or written - no wandering (plan §8), no foreign-queue/patrol fallback (plan §9).
    // Used directly by shadow-mode comparison and tests; NavmeshChooseDirection (below) is the real
    // production entry point once a call site actually drives movement with it.
    Direction NavmeshTableLookup(const TileCoordsXYZ& loc, const Navigation::NavGoalId& goalId);

    // Full production replacement for GuestPathfinding.cpp's ChooseDirection(): table lookup plus
    // the wandering mechanism (reads/writes peep.PathfindHistory exactly as the old algorithm did,
    // restricted to near-optimal alternatives via the table's kBestMask - plan §8) plus a bounded
    // local-search fallback (plan §9) when the precomputed hop would cross a foreign queue or leave
    // a staff patrol area. `rawGoal` is the raw tile goal the old call sites already compute (still
    // needed for PathfindGoal bookkeeping and as the fallback search's target).
    Direction NavmeshChooseDirection(
        const TileCoordsXYZ& loc, const Navigation::NavGoalId& goalId, const TileCoordsXYZ& rawGoal, Peep& peep,
        bool ignoreForeignQueues, RideId queueRideIndex);

    // Computes goalId's table lookup and compares it against `oldResult` (the real ChooseDirection()
    // result already used for movement this call, unaffected by this function). Logs a divergence via
    // LOG_INFO when they differ. No-op unless IsShadowModeEnabled(). Never mutates `peep`.
    void ShadowCompareChooseDirection(
        const Peep& peep, const TileCoordsXYZ& loc, const Navigation::NavGoalId& goalId, Direction oldResult);

    bool IsShadowModeEnabled();
    void SetShadowModeEnabled(bool enabled);

    // Phase B: whether guest call sites use NavmeshChooseDirection's result to actually drive
    // movement, instead of just shadow-comparing it. Independent of shadow mode (both can be on/off
    // in any combination, though shadow mode is redundant once live mode is authoritative - it's
    // still useful to keep computing+logging old-vs-new for regression-watching after the switch).
    bool IsLiveModeEnabled();
    void SetLiveModeEnabled(bool enabled);

} // namespace OpenRCT2::PathFinding
