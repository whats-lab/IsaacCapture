// SPDX-FileCopyrightText: Copyright (c) 2026 WHATs LAB Corp. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "airglove_plugin.hpp"

#include <log_bridge/logger.hpp>
#include <oxr_utils/math.hpp>

#include <airglove_client.h>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <vector>

namespace plugins
{
namespace airglove
{

namespace
{

static_assert(AGC_JOINT_COUNT == XR_HAND_JOINT_COUNT_EXT, "airglove_client reports every XrHandJointEXT joint, in order");

constexpr XrSpaceLocationFlags kPoseValidFlags =
    XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
constexpr XrSpaceLocationFlags kPoseTrackedFlags =
    XR_SPACE_LOCATION_POSITION_TRACKED_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;

constexpr XrPosef kLeftAimToWrist = { { 0.26388208f, 0.17382305f, -0.06730102f, 0.94637327f },
                                      { -0.01391519f, -0.10860867f, 0.08197439f } };
constexpr XrPosef kRightAimToWrist = { { 0.26388208f, -0.17382305f, 0.06730102f, 0.94637327f },
                                       { 0.01391519f, -0.10860867f, 0.08197439f } };

constexpr auto kFramePeriod = std::chrono::milliseconds(16);

XrPosef pose_from_env(const char* name, const XrPosef& fallback)
{
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0')
    {
        return fallback;
    }
    auto logger = isaaccapture::Logger::get("isaaccapture.plugins.airglove.main");
    XrPosef pose{};
    if (std::sscanf(value, "%f,%f,%f,%f,%f,%f,%f", &pose.position.x, &pose.position.y, &pose.position.z,
                    &pose.orientation.x, &pose.orientation.y, &pose.orientation.z, &pose.orientation.w) != 7)
    {
        logger->warn("could not parse {} ('{}', want px,py,pz,qx,qy,qz,qw); using the built-in offset", name, value);
        return fallback;
    }
    const float norm = std::sqrt(pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
                                 pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);
    if (!std::isfinite(norm) || norm < 1e-6f || !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z))
    {
        logger->warn("{} ('{}') is not a valid pose; using the built-in offset", name, value);
        return fallback;
    }
    pose.orientation.x /= norm;
    pose.orientation.y /= norm;
    pose.orientation.z /= norm;
    pose.orientation.w /= norm;
    return pose;
}

plugin_utils::WristSourceMode wrist_source_mode_from_env()
{
    const char* value = std::getenv("AIRGLOVE_WRIST_SOURCE");
    if (value == nullptr || *value == '\0' || std::strcmp(value, "auto") == 0)
    {
        return plugin_utils::WristSourceMode::Auto;
    }
    if (std::strcmp(value, "hand_tracking") == 0)
    {
        return plugin_utils::WristSourceMode::HandTracking;
    }
    if (std::strcmp(value, "controller") == 0)
    {
        return plugin_utils::WristSourceMode::Controller;
    }
    isaaccapture::Logger::get("isaaccapture.plugins.airglove.main")
        ->warn("unknown AIRGLOVE_WRIST_SOURCE '{}', using 'auto'", value);
    return plugin_utils::WristSourceMode::Auto;
}

const char* side_name(XrHandEXT hand)
{
    return hand == XR_HAND_LEFT_EXT ? "left" : "right";
}

} // namespace

AirGlovePlugin::AirGlovePlugin(const std::string& plugin_root_id, const AirGloveOptions& options) noexcept(false)
    : m_options(options), m_root_id(plugin_root_id)
{
    m_logger->info("Initializing with root: {}", m_root_id);
    if (agc_abi_version() != AGC_ABI_VERSION)
    {
        throw std::runtime_error("airglove_client ABI " + std::to_string(agc_abi_version()) + ", plugin built for " +
                                 std::to_string(AGC_ABI_VERSION));
    }

    plugin_utils::WristSourceConfig wrist_config;
    wrist_config.mode = wrist_source_mode_from_env();
    wrist_config.left_aim_to_wrist = pose_from_env("AIRGLOVE_AIM_TO_WRIST_LEFT", kLeftAimToWrist);
    wrist_config.right_aim_to_wrist = pose_from_env("AIRGLOVE_AIM_TO_WRIST_RIGHT", kRightAimToWrist);
    auto wrist_requirements = plugin_utils::WristPoseSource::collect_requirements(wrist_config.mode);

    std::vector<std::shared_ptr<core::ITracker>> trackers = wrist_requirements.trackers;
    auto extensions = core::DeviceIOSession::get_required_extensions(trackers);
    extensions.push_back(XR_NVX1_DEVICE_INTERFACE_BASE_EXTENSION_NAME);
    extensions.insert(extensions.end(), wrist_requirements.extensions.begin(), wrist_requirements.extensions.end());

    m_session = std::make_shared<core::OpenXRSession>("AirGlove", extensions);
    const auto handles = m_session->get_handles();
    m_deviceio_session = core::DeviceIOSession::run(trackers, handles);
    m_time_converter.emplace(handles);
    m_wrist_source = std::make_unique<plugin_utils::WristPoseSource>(
        wrist_config, handles, m_deviceio_session.get(), wrist_requirements.controller_tracker);

    m_client = agc_create(
        m_options.listen_address.c_str(), m_options.listen_port, m_options.spine_address.c_str(), m_options.spine_port);
    if (m_client == nullptr)
    {
        throw std::runtime_error(std::string("airglove_client: ") + agc_last_error());
    }
    agc_set_stale_threshold(m_client, std::chrono::duration<double>(m_options.stale_threshold).count());
    m_logger->info(
        "airglove_client {} listening on UDP {}:{}", agc_version(), m_options.listen_address, m_options.listen_port);

    m_running = true;
    m_worker_thread = std::thread(&AirGlovePlugin::worker_thread, this);
    m_logger->info("AirGlovePlugin initialized and running");
}

AirGlovePlugin::~AirGlovePlugin()
{
    m_logger->info("Shutting down...");
    m_running = false;
    if (m_worker_thread.joinable())
    {
        m_worker_thread.join();
    }
    agc_destroy(m_client);
}

bool AirGlovePlugin::is_running() const noexcept
{
    return m_running.load(std::memory_order_acquire);
}

bool AirGlovePlugin::has_failed() const noexcept
{
    return m_failed.load(std::memory_order_acquire);
}

void AirGlovePlugin::pump_hand(std::unique_ptr<plugin_utils::HandInjector>& injector,
                               XrHandEXT hand,
                               bool& was_active,
                               XrTime time)
{
    std::array<float, AGC_HAND_FLOATS> raw{};
    std::array<uint8_t, AGC_JOINT_COUNT> valid{};
    double age_s = 0.0;
    const int side = hand == XR_HAND_LEFT_EXT ? AGC_LEFT : AGC_RIGHT;
    const bool fresh =
        agc_get_hand(m_client, side, raw.data(), AGC_HAND_FLOATS, valid.data(), nullptr, nullptr, &age_s) == AGC_FRESH &&
        age_s < std::chrono::duration<double>(m_options.stale_threshold).count();
    if (fresh != was_active)
    {
        m_logger->info("{} hand {}", side_name(hand), fresh ? "receiving" : "stale; injection stopped");
        was_active = fresh;
    }
    if (!fresh)
    {
        injector.reset();
        return;
    }
    if (!injector)
    {
        const auto handles = m_session->get_handles();
        injector = std::make_unique<plugin_utils::HandInjector>(handles.instance, handles.session, hand, handles.space);
    }

    plugin_utils::WristSample wrist;
    if (m_wrist_source)
    {
        wrist = m_wrist_source->query(hand == XR_HAND_LEFT_EXT, time);
    }

    std::array<XrHandJointLocationEXT, XR_HAND_JOINT_COUNT_EXT> joints{};
    for (size_t j = 0; j < joints.size(); ++j)
    {
        const float* v = raw.data() + j * AGC_JOINT_FLOATS;
        XrHandJointLocationEXT& joint = joints[j];
        joint.pose.orientation = XrQuaternionf{ v[0], v[1], v[2], v[3] };
        joint.pose.position = XrVector3f{ v[4], v[5], v[6] };
        joint.radius = v[7];
        joint.locationFlags = valid[j] ? kPoseValidFlags : 0;
        if (joint.locationFlags != 0 && wrist.valid)
        {
            joint.pose = oxr_utils::multiply_poses(wrist.pose, joint.pose);
            if (wrist.tracked)
            {
                joint.locationFlags |= kPoseTrackedFlags;
            }
        }
    }
    injector->push(joints.data(), time);
}

void AirGlovePlugin::worker_thread()
{
    while (m_running)
    {
        try
        {
            m_deviceio_session->update();
        }
        catch (const std::exception& e)
        {
            m_logger->error("update error: {}", e.what());
            m_left_injector.reset();
            m_right_injector.reset();
            m_failed.store(true, std::memory_order_release);
            m_running.store(false, std::memory_order_release);
            return;
        }
        const XrTime time = m_time_converter->os_monotonic_now();
        pump_hand(m_left_injector, XR_HAND_LEFT_EXT, m_left_active, time);
        pump_hand(m_right_injector, XR_HAND_RIGHT_EXT, m_right_active, time);
        std::this_thread::sleep_for(kFramePeriod);
    }
    m_left_injector.reset();
    m_right_injector.reset();
}

} // namespace airglove
} // namespace plugins
