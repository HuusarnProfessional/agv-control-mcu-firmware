#include "debug_state.hpp"

#include "../../motion_mcu_routes.hpp"

namespace
{
    motion_mcu_debug_state::encoder_debug_state g_encoder_debug = {};
    motion_mcu_debug_state::imu_debug_state g_imu_debug = {};
    motion_mcu_debug_state::obstacle_debug_state g_obstacle_debug = {};
    motion_mcu_debug_state::voltage_debug_state g_voltage_debug = {};
    motion_mcu_debug_state::motion_debug_state g_motion_debug = {};
    motion_mcu_debug_state::local_position_model_debug_state g_local_position_model_debug = {};

    void clear_stream_state(std::uint8_t payload_id)
    {
        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::encoder_debug))
        {
            const bool stream_enabled = g_encoder_debug.stream_enabled;
            g_encoder_debug = {};
            g_encoder_debug.stream_enabled = stream_enabled;
            return;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::imu_debug))
        {
            const bool stream_enabled = g_imu_debug.stream_enabled;
            g_imu_debug = {};
            g_imu_debug.stream_enabled = stream_enabled;
            return;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::obstacle_debug))
        {
            const bool stream_enabled = g_obstacle_debug.stream_enabled;
            g_obstacle_debug = {};
            g_obstacle_debug.stream_enabled = stream_enabled;
            return;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::voltage_debug))
        {
            const bool stream_enabled = g_voltage_debug.stream_enabled;
            g_voltage_debug = {};
            g_voltage_debug.stream_enabled = stream_enabled;
            return;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::motion_debug))
        {
            const bool stream_enabled = g_motion_debug.stream_enabled;
            g_motion_debug = {};
            g_motion_debug.stream_enabled = stream_enabled;
            return;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::local_position_model_debug))
        {
            const bool stream_enabled = g_local_position_model_debug.stream_enabled;
            g_local_position_model_debug = {};
            g_local_position_model_debug.stream_enabled = stream_enabled;
        }
    }
}

namespace motion_mcu_debug_state
{
    void init()
    {
        g_encoder_debug = {};
        g_imu_debug = {};
        g_obstacle_debug = {};
        g_voltage_debug = {};
        g_motion_debug = {};
        g_local_position_model_debug = {};
    }

    void set_stream_enabled(std::uint8_t payload_id, bool is_enabled)
    {
        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::encoder_debug))
        {
            g_encoder_debug.stream_enabled = is_enabled;
            clear_stream_state(payload_id);
            return;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::imu_debug))
        {
            g_imu_debug.stream_enabled = is_enabled;
            clear_stream_state(payload_id);
            return;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::obstacle_debug))
        {
            g_obstacle_debug.stream_enabled = is_enabled;
            clear_stream_state(payload_id);
            return;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::voltage_debug))
        {
            g_voltage_debug.stream_enabled = is_enabled;
            clear_stream_state(payload_id);
            return;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::motion_debug))
        {
            g_motion_debug.stream_enabled = is_enabled;
            clear_stream_state(payload_id);
            return;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::local_position_model_debug))
        {
            g_local_position_model_debug.stream_enabled = is_enabled;
            clear_stream_state(payload_id);
        }
    }

    bool is_stream_enabled(std::uint8_t payload_id)
    {
        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::encoder_debug))
        {
            return g_encoder_debug.stream_enabled;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::imu_debug))
        {
            return g_imu_debug.stream_enabled;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::obstacle_debug))
        {
            return g_obstacle_debug.stream_enabled;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::voltage_debug))
        {
            return g_voltage_debug.stream_enabled;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::motion_debug))
        {
            return g_motion_debug.stream_enabled;
        }

        if (payload_id == static_cast<std::uint8_t>(motion_mcu_routes::incoming_payload_id::local_position_model_debug))
        {
            return g_local_position_model_debug.stream_enabled;
        }

        return false;
    }

    void set_encoder_debug(const encoder_debug_state &state)
    {
        g_encoder_debug = state;
    }

    void set_imu_debug(const imu_debug_state &state)
    {
        g_imu_debug = state;
    }

    void set_obstacle_debug(const obstacle_debug_state &state)
    {
        g_obstacle_debug = state;
    }

    void set_voltage_debug(const voltage_debug_state &state)
    {
        g_voltage_debug = state;
    }

    void set_motion_debug(const motion_debug_state &state)
    {
        g_motion_debug = state;
    }

    void set_local_position_model_debug(const local_position_model_debug_state &state)
    {
        g_local_position_model_debug = state;
    }

    encoder_debug_state get_encoder_debug()
    {
        return g_encoder_debug;
    }

    imu_debug_state get_imu_debug()
    {
        return g_imu_debug;
    }

    obstacle_debug_state get_obstacle_debug()
    {
        return g_obstacle_debug;
    }

    voltage_debug_state get_voltage_debug()
    {
        return g_voltage_debug;
    }

    motion_debug_state get_motion_debug()
    {
        return g_motion_debug;
    }

    local_position_model_debug_state get_local_position_model_debug()
    {
        return g_local_position_model_debug;
    }
}
