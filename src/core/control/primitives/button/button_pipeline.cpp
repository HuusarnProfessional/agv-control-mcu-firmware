#include "button_pipeline.hpp"

#include <Arduino.h>

#include "../../../mission/mission_buffer.hpp"
#include "../../../mission/mission_runner.hpp"

namespace
{
    constexpr std::uint32_t debounce_ms = 50U;

    bool initialized = false;
    std::uint8_t start_button_pin = 0U;
    int start_button_active_level = HIGH;
    bool last_raw_pressed = false;
    bool debounced_pressed = false;
    std::uint32_t last_change_ms = 0U;

    bool start_button_is_pressed()
    {
        return digitalRead(start_button_pin) == start_button_active_level;
    }

    void start_active_mission()
    {
        (void)mission_runner::start_mission(mission_buffer::active_mission_id());
    }
}

namespace button_pipeline
{
    void init(std::uint8_t start_mission_button_pin, int active_level)
    {
        start_button_pin = start_mission_button_pin;
        start_button_active_level = active_level;
        last_raw_pressed = start_button_is_pressed();
        debounced_pressed = last_raw_pressed;
        last_change_ms = 0U;
        initialized = true;
    }

    void tick(std::uint32_t now_ms)
    {
        if (initialized == false)
        {
            return;
        }

        const bool raw_pressed = start_button_is_pressed();

        if (raw_pressed != last_raw_pressed)
        {
            last_raw_pressed = raw_pressed;
            last_change_ms = now_ms;
            return;
        }

        if ((now_ms - last_change_ms) < debounce_ms)
        {
            return;
        }

        if (raw_pressed == debounced_pressed)
        {
            return;
        }

        debounced_pressed = raw_pressed;

        if (debounced_pressed == false)
        {
            return;
        }

        start_active_mission();
    }
}
