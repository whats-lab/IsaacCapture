// SPDX-FileCopyrightText: Copyright (c) 2026 WHATs LAB Corp. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include "airglove_plugin.hpp"

#include <log_bridge/logger.hpp>

#include <atomic>
#include <charconv>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <thread>

using namespace plugins::airglove;

static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "lock-free atomic bool is required for signal safety");

namespace
{

std::atomic<bool> g_stop_requested{ false };

extern "C" void signal_handler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM)
    {
        g_stop_requested.store(true, std::memory_order_relaxed);
    }
}

void usage(const char* argv0)
{
    const AirGloveOptions defaults;
    std::cerr << "Usage: " << argv0 << " [options]\n\n"
              << "Receives AirGlove hand joints from the Spine app and injects them as OpenXR hand tracking.\n\n"
              << "Options:\n"
              << "  --address=ADDR      Interface to receive on, literal IPv4 (default: " << defaults.listen_address
              << ")\n"
              << "  --port=N            UDP port to receive on (default: " << defaults.listen_port << ")\n"
              << "  --spine-address=A   Spine app address (default: " << defaults.spine_address << ")\n"
              << "  --spine-port=N      Spine app request port (default: " << defaults.spine_port << ")\n"
              << "  --stale-ms=N     Drop a hand after N ms without data (default: " << defaults.stale_threshold.count()
              << ")\n"
              << "  --help           Show this message\n\n"
              << "Environment: AIRGLOVE_WRIST_SOURCE=auto|hand_tracking|controller,\n"
              << "             AIRGLOVE_AIM_TO_WRIST_{LEFT,RIGHT}=px,py,pz,qx,qy,qz,qw\n";
}

bool starts_with(std::string_view text, std::string_view prefix)
{
    return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

bool parse_positive(std::string_view text, unsigned long max, unsigned long& out)
{
    const char* end = text.data() + text.size();
    const auto [ptr, error] = std::from_chars(text.data(), end, out);
    return error == std::errc{} && ptr == end && out > 0 && out <= max;
}

enum class ParseOutcome
{
    Ok,
    HelpRequested,
    Error,
};

ParseOutcome parse_options(int argc, char** argv, std::string& root_id, AirGloveOptions& out, std::string& error)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string_view arg = argv[i];
        unsigned long value = 0;
        if (arg == "--help" || arg == "-h")
        {
            return ParseOutcome::HelpRequested;
        }
        if (arg == "--plugin-root-id")
        {
            if (i + 1 >= argc)
            {
                error = "--plugin-root-id requires a value";
                return ParseOutcome::Error;
            }
            root_id = argv[++i];
        }
        else if (starts_with(arg, "--plugin-root-id="))
        {
            root_id = std::string(arg.substr(17));
        }
        else if (starts_with(arg, "--address="))
        {
            out.listen_address = std::string(arg.substr(10));
        }
        else if (starts_with(arg, "--spine-address="))
        {
            out.spine_address = std::string(arg.substr(16));
        }
        else if (starts_with(arg, "--port="))
        {
            if (!parse_positive(arg.substr(7), 65535, value))
            {
                error = "invalid --port '" + std::string(arg.substr(7)) + "'";
                return ParseOutcome::Error;
            }
            out.listen_port = static_cast<uint16_t>(value);
        }
        else if (starts_with(arg, "--spine-port="))
        {
            if (!parse_positive(arg.substr(13), 65535, value))
            {
                error = "invalid --spine-port '" + std::string(arg.substr(13)) + "'";
                return ParseOutcome::Error;
            }
            out.spine_port = static_cast<uint16_t>(value);
        }
        else if (starts_with(arg, "--stale-ms="))
        {
            if (!parse_positive(arg.substr(11), 60000, value))
            {
                error = "invalid --stale-ms '" + std::string(arg.substr(11)) + "'";
                return ParseOutcome::Error;
            }
            out.stale_threshold = std::chrono::milliseconds(value);
        }
        else
        {
            error = "unknown option '" + std::string(arg) + "'";
            return ParseOutcome::Error;
        }
    }
    return ParseOutcome::Ok;
}

} // namespace

int main(int argc, char** argv)
try
{
    std::string plugin_root_id = "airglove";
    AirGloveOptions options;
    std::string parse_error;
    switch (parse_options(argc, argv, plugin_root_id, options, parse_error))
    {
    case ParseOutcome::HelpRequested:
        usage(argv[0]);
        return 0;
    case ParseOutcome::Error:
        std::cerr << argv[0] << ": " << parse_error << "\n\n";
        usage(argv[0]);
        return 1;
    case ParseOutcome::Ok:
        break;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto logger = isaaccapture::Logger::get("isaaccapture.plugins.airglove.main");
    logger->info("AirGlove Plugin");
    logger->info("Plugin Root ID: {}", plugin_root_id);

    auto plugin = std::make_unique<AirGlovePlugin>(plugin_root_id, options);

    logger->info("Plugin running. Press Ctrl+C to stop.");
    while (!g_stop_requested.load(std::memory_order_relaxed) && plugin->is_running())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const bool failed = plugin->has_failed();
    return failed ? 1 : 0;
}
catch (const std::exception& e)
{
    auto logger = isaaccapture::Logger::get("isaaccapture.plugins.airglove.main");
    logger->error("{}: {}", argv[0], e.what());
    return 1;
}
catch (...)
{
    auto logger = isaaccapture::Logger::get("isaaccapture.plugins.airglove.main");
    logger->error("{}: Unknown error occurred", argv[0]);
    return 1;
}
