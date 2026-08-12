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

namespace OpenRCT2
{
    struct Guest;
    struct Peep;
    struct PathElement;
} // namespace OpenRCT2

namespace OpenRCT2::PathFinding
{
    // maxTilesCheckedOverride: 0 (default) uses the normal staff/guest budget (50000/15000 tiles).
    // A positive value overrides it - used by NavmeshPathfinding's bounded local-search fallback
    // (plan §9), which only needs to resolve a small local ambiguity (a foreign queue or patrol
    // boundary immediately ahead), not a park-spanning route.
    Direction ChooseDirection(
        const TileCoordsXYZ& loc, const TileCoordsXYZ& goal, Peep& peep, bool ignoreForeignQueues, RideId queueRideIndex,
        int32_t maxTilesCheckedOverride = 0);

    // Exposed for OpenRCT2::Navigation's graph builder, which reuses this exact junction
    // classification so navmesh nodes match what the (fallback) heuristic search considers a junction.
    bool PathIsThinJunction(PathElement* path, const TileCoordsXYZ& loc);

    // Exposed for OpenRCT2::Navigation's graph builder, to determine which raw edges a banner
    // blocks for guests (ignoreBanners=false) vs. staff (ignoreBanners=true).
    int32_t PathGetPermittedEdges(bool ignoreBanners, const PathElement* pathElement);

    int32_t CalculateNextDestination(Guest& peep);

    int32_t GuestPathFindParkEntranceEntering(Peep& peep, uint8_t edges);

    int32_t GuestPathFindPeepSpawn(Peep& peep, uint8_t edges);

    int32_t GuestPathFindParkEntranceLeaving(Peep& peep, uint8_t edges);

} // namespace OpenRCT2::PathFinding
