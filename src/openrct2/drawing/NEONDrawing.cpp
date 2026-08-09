/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "../core/Guard.hpp"
#include "Drawing.Sprite.h"
#include "PaletteIndex.h"

using OpenRCT2::Drawing::PaletteIndex;

#ifdef __ARM_NEON

    #include <arm_neon.h>

void MaskNeon(
    int32_t width, int32_t height, const uint8_t* RESTRICT maskSrc, const uint8_t* RESTRICT colourSrc,
    PaletteIndex* RESTRICT dst, int32_t maskWrap, int32_t colourWrap, int32_t dstWrap)
{
    if (width == 32)
    {
        for (int32_t yy = 0; yy < height; yy++)
        {
            int32_t colourStep = yy * (colourWrap + 32);
            int32_t maskStep = yy * (maskWrap + 32);
            int32_t dstStep = yy * (dstWrap + 32);

            // first half
            const uint8x16_t colour1 = vld1q_u8(colourSrc + colourStep);
            const uint8x16_t mask1 = vld1q_u8(maskSrc + maskStep);
            const uint8x16_t dest1 = vld1q_u8(reinterpret_cast<const uint8_t*>(dst) + dstStep);
            const uint8x16_t mc1 = vandq_u8(colour1, mask1);
            const uint8x16_t saturate1 = vceqq_u8(mc1, vdupq_n_u8(0));
            const uint8x16_t blended1 = vbslq_u8(saturate1, dest1, mc1);

            // second half
            const uint8x16_t colour2 = vld1q_u8(colourSrc + 16 + colourStep);
            const uint8x16_t mask2 = vld1q_u8(maskSrc + 16 + maskStep);
            const uint8x16_t dest2 = vld1q_u8(reinterpret_cast<const uint8_t*>(dst) + 16 + dstStep);
            const uint8x16_t mc2 = vandq_u8(colour2, mask2);
            const uint8x16_t saturate2 = vceqq_u8(mc2, vdupq_n_u8(0));
            const uint8x16_t blended2 = vbslq_u8(saturate2, dest2, mc2);

            vst1q_u8(reinterpret_cast<uint8_t*>(dst) + dstStep, blended1);
            vst1q_u8(reinterpret_cast<uint8_t*>(dst) + 16 + dstStep, blended2);
        }
    }
    else
    {
        MaskScalar(width, height, maskSrc, colourSrc, dst, maskWrap, colourWrap, dstWrap);
    }
}

#else

void MaskNeon(
    int32_t width, int32_t height, const uint8_t* RESTRICT maskSrc, const uint8_t* RESTRICT colourSrc,
    PaletteIndex* RESTRICT dst, int32_t maskWrap, int32_t colourWrap, int32_t dstWrap)
{
    OpenRCT2::Guard::Fail("NEON function called on a CPU that doesn't support NEON");
}

#endif // __ARM_NEON
