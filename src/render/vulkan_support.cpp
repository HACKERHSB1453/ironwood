// SPDX-License-Identifier: GPL-3.0-or-later
#include "render/vulkan_support.hpp"

#include <QByteArrayList>
#include <QCoreApplication>
#include <QVulkanInstance>
#include <memory>

namespace iw::render {

namespace {
std::unique_ptr<QVulkanInstance> g_instance;
bool g_validation = false;
} // namespace

QVulkanInstance* createVulkanInstance(bool forceValidation) {
    if (g_instance) return g_instance.get();
    auto inst = std::make_unique<QVulkanInstance>();
#ifdef NDEBUG
    const bool wantValidation = forceValidation;
#else
    const bool wantValidation = true;
    (void)forceValidation;
#endif
    if (wantValidation) {
        const QByteArray layer = "VK_LAYER_KHRONOS_validation";
        for (const auto& l : inst->supportedLayers()) {
            if (l.name == layer) {
                inst->setLayers({layer});
                g_validation = true;
                break;
            }
        }
    }
    if (!inst->create()) return nullptr;
    g_instance = std::move(inst);
    // Destroy with the application object: at static-destruction time the Vulkan loader/validation layer is
    // already being torn down and vkDestroyInstance crashes.
    qAddPostRoutine([] { g_instance.reset(); });
    return g_instance.get();
}

bool validationLayerEnabled() noexcept { return g_validation; }

} // namespace iw::render
