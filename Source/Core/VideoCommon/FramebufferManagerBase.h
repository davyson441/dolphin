// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <memory>

#include "Common/CommonTypes.h"
#include "VideoCommon/TextureConfig.h"

class FramebufferManagerBase
{
public:
  virtual ~FramebufferManagerBase() = default;
  static unsigned int GetEFBLayers() { return 1; }
  static AbstractTextureFormat GetEFBDepthFormat();
};

extern std::unique_ptr<FramebufferManagerBase> g_framebuffer_manager;
