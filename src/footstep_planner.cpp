/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#include <ow_fs_planner/footstep_planner.h>

namespace ow_fs_planner
{

  FootstepPlanner::FootstepPlanner() : Base("FootstepPlanner"),
                                       fixed_plan_(false),
                                       initialized_(false),
                                       max_fwd_step_size_(0.0),
                                       max_bwd_step_size_(0.0),
                                       max_side_step_size_(0.0),
                                       max_step_angle_(0.0),
                                       feet_separation_(0.0),
                                       number_of_steps_(0),
                                       fwd_step_size_cmd_(0.0),
                                       lateral_disp_cmd_(0.0),
                                       step_angle_cmd_(0.0),
                                       cmd_(ow::CartesianVelocity::Zero())
  {
  }

  FootstepPlanner::~FootstepPlanner()
  {
  }

  bool FootstepPlanner::init(const ow::Parameter &parameter, ros::NodeHandle &nh)
  {
    // build the configuration
    parameter_.add<ow::Scalar>("feet_separation", 0.15);
    parameter_.add<ow::Scalar>("max_fwd_step_size", 0.2);
    parameter_.add<ow::Scalar>("max_bwd_step_size", 0.1);
    parameter_.add<ow::Scalar>("max_side_step_size", 0.1);
    parameter_.add<ow::Scalar>("max_step_angle", 0.35);
    if (!parameter_.load(nh, "footstep_planner"))
    {
      ROS_ERROR("%s::init: Config loading failed.", Base::name().c_str());
      return false;
    }

    // load module parameter
    parameter_.get("feet_separation", feet_separation_);
    parameter_.get("max_fwd_step_size", max_fwd_step_size_);
    parameter_.get("max_bwd_step_size", max_bwd_step_size_);
    parameter_.get("max_side_step_size", max_side_step_size_);
    parameter_.get("max_step_angle", max_step_angle_);

    // init members
    if (!single_planner_.init(parameter_))
    {
      ROS_ERROR("%s::init: Failed to init SinglePlanner", Base::name().c_str());
    }

    ow::HomogeneousTransformation T_l_w, T_r_w;
    T_l_w.setIdentity();
    T_r_w.setIdentity();
    T_l_w.pos().y() = feet_separation_ / 2.0;
    T_r_w.pos().y() = -feet_separation_ / 2.0;
    start(T_l_w, T_r_w);

    return true;
  }

  const ow::FootStepList& FootstepPlanner::start(
      const ow::CartesianPosition &X_l_w,
      const ow::CartesianPosition &X_r_w)
  {
    ow::FootStepList foot_steps(2); 
    foot_steps[0].pos() = X_r_w;
    foot_steps[0].footId() = ow::FootId::RIGHT;
    foot_steps[1].pos() = X_l_w;
    foot_steps[1].footId() = ow::FootId::LEFT;
    return reset(foot_steps);
  }

  const ow::FootStepList& FootstepPlanner::reset( const ow::FootStepList& foot_steps)
  {
    if(foot_steps.size() < 2)
    {
      ROS_ERROR_STREAM("FootstepPlanner::reset: Requires last left and right step");
      return foot_steps;
    }

    // copy inital footsteps, reset the current step to zero and one
    plan_ = foot_steps;
    plan_[ow::FootId::LEFT].nStep() = 0;
    plan_[ow::FootId::LEFT].finalStep() = false;
    plan_[ow::FootId::RIGHT].nStep() = 1;
    plan_[ow::FootId::RIGHT].finalStep() = false;
    cur_step_id_ = 1;

    initialized_ = true;
    return plan_;
  }

  void FootstepPlanner::abort(const ow::FootStep &current_step)
  {
    if(plan_.size() < current_step.nStep() + 3)
    {
      // we are about to finish anyway, nothing to do
      return;
    }

    // Generate final step
    plan_.at(current_step.nStep() + 2) =
        single_planner_.planFinalStep(plan_.at(current_step.nStep() + 1));

    // remove the rest after final
    plan_.resize(current_step.nStep() + 3);
    number_of_steps_ = plan_.size();
  }

  int FootstepPlanner::update(const ow::FootStep &current_step)
  {
    if (!initialized_)
    {
      ROS_ERROR_STREAM("Footstep planner is not initialized yet.");
      return -3;
    }

    if (current_step.footId() == plan_[current_step.nStep()].footId())
    {
      if (cur_step_id_ < plan_.size())
      {
        if (!fixed_plan_)
        {
          //ROS_WARN_STREAM("Planning joystick.");
          planSteps(current_step, cmd_);
        }
        else
        {
          //ROS_WARN_STREAM("Planning fixed");
          planSteps(current_step, fwd_step_size_cmd_, lateral_disp_cmd_, step_angle_cmd_);
        }
      }
      else
      {
        ROS_WARN_STREAM("Current step exceeds plan");
        return -2;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Received foostep's 'foot' does not match the plan.");
      return -1;
    }
    return 0;
  }

  const ow::FootStepList &FootstepPlanner::generateFixedPlan(
      ow::Scalar fwd_step_size,
      ow::Scalar lateral_disp,
      ow::Scalar step_angle,
      size_t number_of_steps)
  {
    fwd_step_size_cmd_ = fwd_step_size;
    lateral_disp_cmd_ = lateral_disp;
    step_angle_cmd_ = step_angle;
    number_of_steps_ = number_of_steps;

    if (initialized_)
    {
      number_of_steps_ = number_of_steps;
      fixed_plan_ = true;
      return planSteps(plan_[cur_step_id_], fwd_step_size, lateral_disp, step_angle);
    }
    else
    {
      plan_.clear();
      return plan_;
    }
  }

  const ow::FootStepList &FootstepPlanner::generateFixedPlan()
  {
    return generateFixedPlan(fwd_step_size_cmd_,
                             lateral_disp_cmd_,
                             step_angle_cmd_,
                             number_of_steps_);
  }

  const ow::FootStepList &FootstepPlanner::planSteps(
      const ow::FootStep& current_step,
      const ow::CartesianVelocity &cmd )
  {
    if (initialized_)
    {
      // clamp the velocity command
      cmd_ = cmd;
      ow::clamp(cmd_, -1.0, 1.0);
      
      // Update current step.
      cur_step_id_ = current_step.nStep();
      plan_[cur_step_id_] = current_step;

      // move the finalstep flag forward
      if(cmd.linear().norm() > 0.1 || std::abs(cmd.angular().z()) > 0.1)
      {
        plan_[plan_.size() - 1].finalStep() = false;

        while ((plan_.size() - cur_step_id_) < 5)
        {
          plan_.push_back(current_step);
          plan_.back().finalStep() = true;
        }
      }
      else
      {
        // If velocity command is zero, we keep the current end as final step
        plan_[plan_.size() - 1].finalStep() = true;
      }

      // Generate horizon of steps.
      for (size_t i = cur_step_id_ + 1; i < plan_.size() - 1; i++)
      {
        plan_[i] = single_planner_.planStep(plan_[i - 1], cmd_);
      }

      // Generate final step
      plan_[plan_.size() - 1] =
          single_planner_.planFinalStep(plan_[plan_.size() - 2]);
    }
    return plan_;
  }

  const ow::FootStepList &FootstepPlanner::planSteps(
      const ow::FootStep& current_step,
      const ow::Scalar fwd_step_size,
      const ow::Scalar lateral_disp,
      const ow::Scalar step_angle)
  {
    /*ROS_INFO_STREAM("Plann command: ");
    ROS_INFO_STREAM("fwd_step_size: " << fwd_step_size);
    ROS_INFO_STREAM("lateral_disp: " << lateral_disp);
    ROS_INFO_STREAM("step_angle: " << step_angle);
    ROS_INFO_STREAM("number_of_steps_: " << number_of_steps_);*/

    // Compute velocity command.
    cmd_ = computeCommand(fwd_step_size, lateral_disp, step_angle);

    // Update current step.
    cur_step_id_ = current_step.nStep();
    plan_[cur_step_id_] = current_step;

    // Resize footstep vector.
    plan_.resize(number_of_steps_, current_step);

    // Generate horizon of steps.
    for (size_t i = cur_step_id_ + 1; i < plan_.size(); i++)
    {
      plan_[i] = single_planner_.planStep(plan_[i - 1], cmd_);
    }

    // Generate final step
    plan_[plan_.size() - 1] =
        single_planner_.planFinalStep(plan_[plan_.size() - 2]);

    return plan_;
  }

  ow::CartesianVelocity FootstepPlanner::computeCommand(
      ow::Scalar fwd_step_size, ow::Scalar lateral_disp, ow::Scalar step_angle)
  {
    ow::CartesianVelocity XP = ow::CartesianVelocity::Zero();

    // create 6d twist from 3d planar twist
    if (fwd_step_size > 0.0)
      XP.linear().x() = fwd_step_size / max_fwd_step_size_;
    else
      XP.linear().x() = fwd_step_size / max_bwd_step_size_;
    XP.linear().y() = lateral_disp / max_side_step_size_;
    XP.angular().z() = step_angle / max_step_angle_;

    ow::clamp(XP, -1.0, 1.0);
    return XP;
  }

  void FootstepPlanner::print()
  {
    for (unsigned int i = 0; i < plan_.size(); i++)
    {
      ROS_INFO_STREAM(" ");
      ROS_INFO_STREAM("Step " << i << ": ");
      ROS_INFO_STREAM("nStep: " << plan_[i].nStep());
      ROS_INFO_STREAM("Pos: " << plan_[i].pos().linear().transpose());
      ROS_INFO_STREAM("Leg: " << plan_[i].footId().toString());
      ROS_INFO_STREAM("stop: " << plan_[i].finalStep());
    }
  }

  const ow::FootStepList &FootstepPlanner::footSteps() const
  {
    return plan_;
  }

} // namespace ow_fs_planner
