#include "../debug_handler_declarations.hpp"

#include <cstddef>
#include <cstdio>

#include "../../../middleware_parse_helpers.hpp"
#include "../../handler_helpers.hpp"
#include "../../../../../position_sensorfusion/filtered_global_position/filtered_global_position.hpp"
#include "debug_handler_helpers.hpp"

namespace debug_handlers
{
    bool handle_set_filtered_global_position_anchor_confidence_gain()
    {
        std::uint16_t gain_permille = 0U;

        if (middleware_parse_helpers::read_uint16_and_end(gain_permille, debug_handler_helpers::timeout_us) == false)
        {
            return debug_handler_helpers::write_bad_format();
        }

        if (filtered_global_position::set_candidate_position_anchor_confidence_gain_permille(gain_permille) == false)
        {
            return handler_helpers::write_response_text("filtered_global_position_anchor_confidence_gain rejected");
        }

        char response[112] = {};
        const int formatted_length = std::snprintf(
            response,
            sizeof(response),
            "filtered_global_position_anchor_confidence_gain %u",
            static_cast<unsigned>(filtered_global_position::get_candidate_position_anchor_confidence_gain_permille()));

        if ((formatted_length <= 0) || (static_cast<std::size_t>(formatted_length) >= sizeof(response)))
        {
            return false;
        }

        return handler_helpers::write_response_text(response);
    }
}
