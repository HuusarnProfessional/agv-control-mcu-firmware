#pragma once

#include <cstdint>

#include "uwb_position_history.hpp"

namespace uwb_position_history_gates
{
    std::uint16_t physical_jump_confidence(const uwb_position_history::history_state &history, const uwb_position_history::sample &new_sample);

    std::uint16_t prediction_confidence(const uwb_position_history::history_state &history, const uwb_position_history::sample &new_sample);

    std::uint16_t hampel_speed_confidence(const uwb_position_history::history_state &history, const uwb_position_history::sample &new_sample);

    std::uint16_t combine_gate_confidence(std::uint16_t physical_confidence, std::uint16_t prediction_confidence, std::uint16_t hampel_confidence);
}