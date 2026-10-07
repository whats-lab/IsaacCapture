// SPDX-FileCopyrightText: Copyright (c) 2026 WHATs LAB Corp. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <deviceio_session/deviceio_session.hpp>
#include <log_bridge/logger.hpp>
#include <openxr/openxr.h>
#include <oxr/oxr_session.hpp>
#include <oxr_utils/oxr_time.hpp>
#include <plugin_utils/hand_injector.hpp>
#include <plugin_utils/wrist_pose_source.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>

struct agc_client;

namespace plugins
{
namespace airglove
{

// Network endpoints of the AirGlove client and the hand staleness window.
struct AirGloveOptions
{
    std::string listen_address = "127.0.0.1";
    uint16_t listen_port = 4040;
    std::string spine_address = "127.0.0.1";
    uint16_t spine_port = 4042;
    std::chrono::milliseconds stale_threshold{ 200 };
};

// AirGlove -> OpenXR hand-tracking device plugin.
//
// Reads the latest wrist-relative 26-joint hands from the prebuilt airglove_client
// library (which receives them from the AirGlove Spine app), places them at a
// plugin_utils::WristPoseSource wrist pose, and injects them into the OpenXR hand
// layer via plugin_utils::HandInjector. The existing core::HandTracker consumes them.
class AirGlovePlugin
{
public:
    AirGlovePlugin(const std::string& plugin_root_id, const AirGloveOptions& options) noexcept(false);
    ~AirGlovePlugin();

    // True while the injection worker is running.
    bool is_running() const noexcept;
    // True when the worker stopped on an unrecoverable error.
    bool has_failed() const noexcept;

    AirGlovePlugin(const AirGlovePlugin&) = delete;
    AirGlovePlugin& operator=(const AirGlovePlugin&) = delete;
    AirGlovePlugin(AirGlovePlugin&&) = delete;
    AirGlovePlugin& operator=(AirGlovePlugin&&) = delete;

private:
    // Pumps both hands every frame until stopped.
    void worker_thread();
    // Push (or reset) one hand's injector from the client's latest hand.
    void pump_hand(std::unique_ptr<plugin_utils::HandInjector>& injector, XrHandEXT hand, bool& was_active, XrTime time);

    std::shared_ptr<spdlog::logger> m_logger = isaaccapture::Logger::get("isaaccapture.plugins.airglove.AirGlovePlugin");
    AirGloveOptions m_options;
    agc_client* m_client = nullptr;
    std::shared_ptr<core::OpenXRSession> m_session;
    std::unique_ptr<core::DeviceIOSession> m_deviceio_session;
    std::unique_ptr<plugin_utils::HandInjector> m_left_injector;
    std::unique_ptr<plugin_utils::HandInjector> m_right_injector;
    std::optional<core::XrTimeConverter> m_time_converter;
    // Declared after m_session/m_deviceio_session: destroyed first, while the
    // XR handles and the (non-owned) DeviceIOSession it references are alive.
    std::unique_ptr<plugin_utils::WristPoseSource> m_wrist_source;
    bool m_left_active = false;
    bool m_right_active = false;
    std::thread m_worker_thread;
    std::atomic<bool> m_running{ false };
    std::atomic<bool> m_failed{ false };
    std::string m_root_id;
};

} // namespace airglove
} // namespace plugins
