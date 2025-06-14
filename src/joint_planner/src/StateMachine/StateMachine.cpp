#include "StateMachine.h"

namespace planner_sm
{
    Status PlannerStateMachine::initial_state(const Event *const event)
    {
        Status status = fsm::Status::IGNORED_STATUS;
        switch (event->signal)
        {
            case Signal::ENTRY_SIG:
            {
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Initial state");
                _start_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::TIMEOUT_3S_SIG:
            {
                /* transient to configuration state */
                _state = (fsm::FSM::StateHandler)&PlannerStateMachine::configuration_state;
                status = fsm::Status::TRAN_STATUS;
                break;
            }
            
        }

        return status;
    }

    Status PlannerStateMachine::configuration_state(const Event *const event)
    {
        Status status = fsm::Status::IGNORED_STATUS;
        switch (event->signal)
        {
            case Signal::ENTRY_SIG:
            {
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Configuration state");
                _start_timer();
                _planner->compute(_sampling_time); // call the compute to get the first position in the trajectory, saved internally
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::TIMEOUT_5MS_SIG:
            {
                /* Send zeros joint positions cmds
                    + Read current joint positions q
                    + q_cmds gradually change from q to q_zeros within 5s
                */
                static bool is_first_time = true;
                static const uint16_t max_counter = static_cast<uint16_t>(10.0 / _sampling_time); // 5s wait
                static std::vector<float> q_zeros = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                static std::vector<float> prev_q_zeros = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                static std::vector<float> q;
                static uint16_t counter = 0;
                if(is_first_time)
                {
                    q = prev_q_zeros; // Read current joint positions
                    _planner->reset_filter(); // Reset the filter in the planner
                    for(int index = 0; index < 3; index++)
                    {
                        q_zeros = _planner->get_actions(); // Get the first joint position commands from the planner
                        q_zeros = _planner->apply_filter(q_zeros); // Apply filter to the joint position commands
                    }
                    
                    prev_q_zeros = q_zeros; // Save the previous joint position commands
                    is_first_time = false;
                }
                
                std::vector<float> q_cmds(5, 0.0f);
                float phase = static_cast<float>(counter) / static_cast<float>(max_counter);

                // Calculate the joint commands using first order interpolation and send them
                std::transform(q.begin(), q.end(), q_zeros.begin(), q_cmds.begin(),
                    [phase](float q_i, float q_zero_i) 
                    {
                        return q_i + (q_zero_i - q_i) * phase;
                    }
                );
                _actuator_interface->send_command(q_cmds); 

                // Increment the counter and check if the maximum counter is reached
                counter++;
                if(counter < max_counter)
                {
                    /* Still in configuration state */
                    status = fsm::Status::HANDLED_STATUS;
                    break;
                }

                /* Configuration is complete */
                counter = 0; // reset the counter
                is_first_time = true; // reset the first time flag

                /* Transient to planning state*/
               _state = (fsm::FSM::StateHandler)&PlannerStateMachine::planning_state;
                status = fsm::Status::TRAN_STATUS;
                break;
            }

        }

        return status;
    }

    Status PlannerStateMachine::planning_state(const Event *const event)
    {
        Status status = fsm::Status::IGNORED_STATUS;
        switch (event->signal)
        {
            case Signal::ENTRY_SIG:
            {
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Planning state");
                _start_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::TIMEOUT_5MS_SIG:
            {
                if(_planner->is_trajectory_completed())
                {
                    /* Planner plan for 1 trajectory is reached, transient to finished state */
                    _state = (fsm::FSM::StateHandler)&PlannerStateMachine::finished_state;
                    status = fsm::Status::TRAN_STATUS;
                    break;
                }

                /* Planning
                    Compute joint commands and send joint commands
                */
                static bool is_first_time = true;
                if(is_first_time)
                {
                    _planner->reset_filter();
                    is_first_time = false;
                }
                std::vector<float> joint_position_cmds = _planner->compute(_sampling_time);
                std::vector<float> filtered_positions_cmds = _planner->apply_filter(joint_position_cmds);

                /* Check valid joint cmds */
                bool is_nan = std::any_of(filtered_positions_cmds.begin(), filtered_positions_cmds.end(), [](float val) { return std::isnan(val); });
                bool is_inf = std::any_of(filtered_positions_cmds.begin(), filtered_positions_cmds.end(), [](float val) { return std::isinf(val); });
                if(is_nan || is_inf)
                {
                    RCLCPP_ERROR(_node->get_logger(), "PlannerStateMachine: Joint position commands contain NaN or Inf values");
                    status = fsm::Status::IGNORED_STATUS;
                    break;
                }

                /* Send the filtered command to the actuator interface */
                _actuator_interface->send_command(filtered_positions_cmds);
                _actuator_interface->send_command_raw(joint_position_cmds);

                status = fsm::Status::HANDLED_STATUS;
                break;
            }
        }

        return status;
    }

    Status PlannerStateMachine::finished_state(const Event *const event)
    {
        Status status = fsm::Status::IGNORED_STATUS;
        switch (event->signal)
        {
            case Signal::ENTRY_SIG:
            {
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Finished state");
                _start_timer();
                _planner->change_trajectory();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::TIMEOUT_1S_SIG:
            {
                /* Transient to configuration state */
                _state = (fsm::FSM::StateHandler)&PlannerStateMachine::configuration_state;
                status = fsm::Status::TRAN_STATUS;
                break;
            }
        }

        return status;
    }
}