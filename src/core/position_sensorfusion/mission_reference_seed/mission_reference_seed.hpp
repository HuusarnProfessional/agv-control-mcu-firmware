#pragma once

#include <cstdint>

#include "../../motion_mcu_communication/state/incoming/incoming_state.hpp"
#include "../global_reference_selector/global_reference_selector.hpp"

namespace mission_reference_seed
{
    void init();

    void reset_runtime_state();

    bool is_pending();

    bool read_seed_heading(std::int32_t &heading_urad_out);

    global_reference_selector::reference_activation update(const motion_mcu_incoming_state::local_position_state &local_position, const global_reference_selector::current_reference_snapshot &current_reference, std::uint32_t now_ms);
}
