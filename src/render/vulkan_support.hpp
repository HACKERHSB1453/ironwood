// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

class QVulkanInstance;

namespace iw::render {

/// Creates (once, process-wide, owned internally) the QVulkanInstance used by the QRhi Vulkan backend.
/// In debug builds (NDEBUG undefined) or when `forceValidation` is true, enables the Khronos validation layer
/// (VK_LAYER_KHRONOS_validation) if the loader reports it; otherwise runs without it (no error).
/// Attach the result with `window->setVulkanInstance(...)` BEFORE the window is shown.
/// Returns nullptr if Vulkan is unavailable (callers fall back to another QRhi backend).
/// UI thread only.
QVulkanInstance* createVulkanInstance(bool forceValidation = false);

/// True if the validation layer was enabled for the instance returned by createVulkanInstance().
bool validationLayerEnabled() noexcept;

} // namespace iw::render
