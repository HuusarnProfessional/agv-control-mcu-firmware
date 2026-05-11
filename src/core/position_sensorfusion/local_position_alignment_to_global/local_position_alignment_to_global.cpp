#include "local_position_alignment_to_global.hpp"

#include "alignment_anchor.hpp"
#include "alignment_confidence.hpp"
#include "alignment_pending_request.hpp"
#include "alignment_projection.hpp"
#include "alignment_request_gate.hpp"

namespace
{
    alignment_anchor::anchor_state active_anchor = {};
    alignment_anchor::anchor_state pending_anchor = {};
    alignment_pending_request::pending_state pending_state = {};

    local_position_alignment_to_global::branch_request build_branch_request(const motion_mcu_incoming_state::local_position_state &local_position, std::uint16_t anchor_score, bool is_upgrade)
    {
        local_position_alignment_to_global::branch_request request = {};

        request.has_request = true;
        request.pose_id = local_position.pose_id;
        request.branch_id = local_position.branch_id;
        request.anchor_score = anchor_score;
        request.is_upgrade = is_upgrade;

        return request;
    }

    void update_pending_state(const motion_mcu_incoming_state::local_position_state &local_position, std::uint32_t now_ms)
    {
        const bool branch_arrived = alignment_pending_request::branch_has_arrived(pending_state, local_position.branch_id);

        if (branch_arrived == true)
        {
            active_anchor = pending_anchor;
            alignment_anchor::set_local_branch_id(active_anchor, local_position.branch_id);
            alignment_anchor::clear(pending_anchor);
            alignment_pending_request::mark_branch_arrived(pending_state, now_ms);

            return;
        }

        alignment_pending_request::update_settling(pending_state, now_ms);
    }
}

namespace local_position_alignment_to_global
{
    void init()
    {
        alignment_anchor::clear(active_anchor);
        alignment_anchor::clear(pending_anchor);
        alignment_pending_request::init(pending_state);
    }

    output_snapshot update(const motion_mcu_incoming_state::local_position_state &local_position, const global_position_heading::output_snapshot &global_position, std::uint32_t now_ms)
    {
        if (local_position.has_pose == false)
        {
            return {};
        }

        update_pending_state(local_position, now_ms);

        const std::uint16_t anchor_score = alignment_confidence::calculate_anchor_score(global_position, now_ms);

        if (active_anchor.valid == false)
        {
            if (anchor_score > 0U)
            {
                active_anchor = alignment_anchor::create_initial(local_position, global_position, anchor_score);
            }

            return alignment_projection::project(active_anchor, local_position);
        }

        output_snapshot output = alignment_projection::project(active_anchor, local_position);
        const alignment_request_gate::request_decision decision = alignment_request_gate::evaluate(pending_state, anchor_score, output.confidence_position, global_position.sample_id, now_ms);

        if (decision.should_request == false)
        {
            return output;
        }

        pending_anchor = alignment_anchor::create_branch(global_position, anchor_score);
        alignment_pending_request::start(pending_state, local_position.pose_id, local_position.branch_id, global_position.sample_id, anchor_score, now_ms);
        output.request = build_branch_request(local_position, anchor_score, decision.is_upgrade);

        return output;
    }

    output_snapshot read_output(const motion_mcu_incoming_state::local_position_state &local_position)
    {
        return alignment_projection::project(active_anchor, local_position);
    }
}