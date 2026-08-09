/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "../DrawingEngineFactory.hpp"

#include <SDL3/SDL.h>
#include <cmath>
#include <memory>
#include <openrct2/Diagnostic.h>
#include <openrct2/Game.h>
#include <openrct2/config/Config.h>
#include <openrct2/core/Guard.hpp>
#include <openrct2/drawing/IDrawingEngine.h>
#include <openrct2/drawing/LightFX.h>
#include <openrct2/drawing/X8DrawingEngine.h>
#include <openrct2/interface/Window.h>
#include <openrct2/paint/Paint.h>
#include <openrct2/ui/UiContext.h>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;
using namespace OpenRCT2::Ui;

// SDL_GPU composite-only engine: the CPU software rasteriser (X8DrawingEngine) draws into
// the indexed-colour _bits buffer exactly as it does for the SOFTWARE_HWD engine; this class
// only replaces the presentation step, uploading the palette-resolved frame to a GPU texture
// and using SDL_BlitGPUTexture to present it. No custom shaders/pipelines are needed for this.
class SDLGPUDrawingEngine final : public X8DrawingEngine
{
private:
    IUiContext& _uiContext;
    SDL_Window* _window = nullptr;
    SDL_GPUDevice* _device = nullptr;

    SDL_GPUTexture* _screenTexture = nullptr;
    SDL_GPUTransferBuffer* _transferBuffer = nullptr;
    uint32_t _screenTextureWidth = 0;
    uint32_t _screenTextureHeight = 0;

    uint32_t _paletteRGBA[256] = { 0 };

    bool _useVsync = true;
    bool _smoothNN = false;

public:
    explicit SDLGPUDrawingEngine(IUiContext& uiContext)
        : X8DrawingEngine(uiContext)
        , _uiContext(uiContext)
    {
        _window = static_cast<SDL_Window*>(_uiContext.GetWindow());
    }

    ~SDLGPUDrawingEngine() override
    {
        if (_device != nullptr)
        {
            ReleaseScreenTexture();
            SDL_ReleaseWindowFromGPUDevice(_device, _window);
            SDL_DestroyGPUDevice(_device);
        }
    }

    void Initialise() override
    {
        _device = SDL_CreateGPUDevice(
            SDL_GPU_SHADERFORMAT_METALLIB | SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_DXIL, false, nullptr);
        Guard::Assert(_device != nullptr, "Failed to create SDL_GPU device: %s", SDL_GetError());

        if (!SDL_ClaimWindowForGPUDevice(_device, _window))
        {
            Guard::Fail("Failed to claim window for SDL_GPU device: %s", SDL_GetError());
        }

        SetVSync(_useVsync);
    }

    void SetVSync(bool vsync) override
    {
        _useVsync = vsync;
        auto presentMode = vsync ? SDL_GPU_PRESENTMODE_VSYNC : SDL_GPU_PRESENTMODE_MAILBOX;
        if (!vsync && !SDL_WindowSupportsGPUPresentMode(_device, _window, presentMode))
        {
            presentMode = SDL_GPU_PRESENTMODE_IMMEDIATE;
            if (!SDL_WindowSupportsGPUPresentMode(_device, _window, presentMode))
            {
                presentMode = SDL_GPU_PRESENTMODE_VSYNC;
            }
        }
        SDL_SetGPUSwapchainParameters(_device, _window, SDL_GPU_SWAPCHAINCOMPOSITION_SDR, presentMode);
    }

    void Resize(uint32_t width, uint32_t height) override
    {
        if (width == 0 || height == 0)
        {
            return;
        }

        ReleaseScreenTexture();

        SDL_GPUTextureCreateInfo textureInfo{};
        textureInfo.type = SDL_GPU_TEXTURETYPE_2D;
        textureInfo.format = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;
        textureInfo.usage = SDL_GPU_TEXTUREUSAGE_SAMPLER;
        textureInfo.width = width;
        textureInfo.height = height;
        textureInfo.layer_count_or_depth = 1;
        textureInfo.num_levels = 1;
        textureInfo.sample_count = SDL_GPU_SAMPLECOUNT_1;

        _screenTexture = SDL_CreateGPUTexture(_device, &textureInfo);
        Guard::Assert(
            _screenTexture != nullptr, "Failed to create screen texture (%ux%u): %s", width, height, SDL_GetError());

        SDL_GPUTransferBufferCreateInfo transferInfo{};
        transferInfo.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
        transferInfo.size = width * height * 4;

        _transferBuffer = SDL_CreateGPUTransferBuffer(_device, &transferInfo);
        Guard::Assert(_transferBuffer != nullptr, "Failed to create transfer buffer: %s", SDL_GetError());

        _screenTextureWidth = width;
        _screenTextureHeight = height;

        auto scaleQuality = GetContext()->GetUiContext().GetScaleQuality();
        _smoothNN = scaleQuality == ScaleQuality::smoothNearestNeighbour;

        X8DrawingEngine::Resize(width, height);
    }

    void SetPalette(const GamePalette& palette) override
    {
        for (int32_t i = 0; i < 256; i++)
        {
            const auto& src = palette[i];
            _paletteRGBA[i] = src.red | (src.green << 8) | (src.blue << 16) | (255u << 24);
        }
    }

    void BeginDraw() override
    {
        X8DrawingEngine::BeginDraw();
    }

    void EndDraw() override
    {
        X8DrawingEngine::EndDraw();

        Display();
    }

private:
    void ReleaseScreenTexture()
    {
        if (_screenTexture != nullptr)
        {
            SDL_ReleaseGPUTexture(_device, _screenTexture);
            _screenTexture = nullptr;
        }
        if (_transferBuffer != nullptr)
        {
            SDL_ReleaseGPUTransferBuffer(_device, _transferBuffer);
            _transferBuffer = nullptr;
        }
    }

    void CopyBitsToTransferBuffer()
    {
        void* pixels = SDL_MapGPUTransferBuffer(_device, _transferBuffer, true);
        if (pixels == nullptr)
        {
            LOG_WARNING("SDLGPUDrawingEngine::CopyBitsToTransferBuffer error: %s", SDL_GetError());
            return;
        }

        auto* dst = static_cast<uint32_t*>(pixels);
        auto* src = _bits;
        for (size_t i = static_cast<size_t>(_screenTextureWidth) * _screenTextureHeight; i > 0; i--)
        {
            *dst++ = _paletteRGBA[EnumValue(*src++)];
        }

        SDL_UnmapGPUTransferBuffer(_device, _transferBuffer);
    }

    void Display()
    {
        if (_screenTexture == nullptr)
        {
            return;
        }

        CopyBitsToTransferBuffer();

        auto* commandBuffer = SDL_AcquireGPUCommandBuffer(_device);
        if (commandBuffer == nullptr)
        {
            LOG_WARNING("SDLGPUDrawingEngine::Display: SDL_AcquireGPUCommandBuffer failed: %s", SDL_GetError());
            return;
        }

        SDL_GPUTextureTransferInfo transferSrc{};
        transferSrc.transfer_buffer = _transferBuffer;
        transferSrc.pixels_per_row = _screenTextureWidth;
        transferSrc.rows_per_layer = _screenTextureHeight;

        SDL_GPUTextureRegion textureDst{};
        textureDst.texture = _screenTexture;
        textureDst.w = _screenTextureWidth;
        textureDst.h = _screenTextureHeight;
        textureDst.d = 1;

        auto* copyPass = SDL_BeginGPUCopyPass(commandBuffer);
        SDL_UploadToGPUTexture(copyPass, &transferSrc, &textureDst, true);
        SDL_EndGPUCopyPass(copyPass);

        SDL_GPUTexture* swapchainTexture = nullptr;
        uint32_t swapchainWidth = 0;
        uint32_t swapchainHeight = 0;
        if (!SDL_WaitAndAcquireGPUSwapchainTexture(commandBuffer, _window, &swapchainTexture, &swapchainWidth, &swapchainHeight)
            || swapchainTexture == nullptr)
        {
            SDL_SubmitGPUCommandBuffer(commandBuffer);
            return;
        }

        SDL_GPUBlitInfo blitInfo{};
        blitInfo.source.texture = _screenTexture;
        blitInfo.source.w = _screenTextureWidth;
        blitInfo.source.h = _screenTextureHeight;
        blitInfo.destination.texture = swapchainTexture;
        blitInfo.destination.w = swapchainWidth;
        blitInfo.destination.h = swapchainHeight;
        blitInfo.load_op = SDL_GPU_LOADOP_DONT_CARE;
        blitInfo.filter = _smoothNN ? SDL_GPU_FILTER_LINEAR : SDL_GPU_FILTER_NEAREST;

        SDL_BlitGPUTexture(commandBuffer, &blitInfo);

        SDL_SubmitGPUCommandBuffer(commandBuffer);
    }
};

std::unique_ptr<IDrawingEngine> Ui::CreateSDLGPUDrawingEngine(IUiContext& uiContext)
{
    return std::make_unique<SDLGPUDrawingEngine>(uiContext);
}
