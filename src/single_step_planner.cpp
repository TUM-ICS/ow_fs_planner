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

#include <ow_fs_planner/single_step_planner.h>

namespace ow_fs_planner
{

  SingleStepPlanner::SingleStepPlanner()
  {
  }

  bool SingleStepPlanner::init(const ow::Parameter &parameter)
  {
    parameter.get("feet_separation", feet_separation_);
    parameter.get("max_fwd_step_size", max_fwd_step_size_);
    parameter.get("max_bwd_step_size", max_bwd_step_size_);
    parameter.get("max_side_step_size", max_side_step_size_);
    parameter.get("max_step_angle", max_step_angle_);

    return true;
  }

  ow::FootStep SingleStepPlanner::planStep(
      const ow::FootStep &prev_step,
      const ow::CartesianVelocity &cmd)
  {
    ow::FootStep step;

    // Compute the relative step pose
    ow::CartesianPosition X_rel = ow::CartesianPosition::Zero();
    if (cmd.linear().x() >= 0.0)
    {
      X_rel.linear().x() = cmd.linear().x() * max_fwd_step_size_;
    }
    else
    {
      X_rel.linear().x() = cmd.linear().x() * max_bwd_step_size_;
    }

    // Next: Right foot.
    if (prev_step.footId() == ow::FootId::LEFT)
    {
      if (cmd.linear().y() >= 0.0)
      {
        // Inward stepping is neglegted
        X_rel.linear().y() = -feet_separation_;
      }
      else
      {
        X_rel.linear().y() = cmd.linear().y() * max_side_step_size_ - feet_separation_;
      }

      // Inward turning ( > 0) is neglected.
      if (cmd.angular().z() < 0.0)
      {
        X_rel.angular() = ow::Rotation3::Rz(max_step_angle_ * cmd.angular().z());
      }

      step.footId() = ow::FootId::RIGHT;
    }
    else
    {
      if (cmd.linear().y() >= 0.0)
      {
        X_rel.linear().y() = cmd.linear().y() * max_side_step_size_ + feet_separation_;
      }
      else
      {
        // Inward stepping is neglegted
        X_rel.linear().y() = feet_separation_;
      }

      // Inward turning ( < 0) is neglected.
      if (cmd.angular().z() > 0.0)
      {
        X_rel.angular() = ow::Rotation3::Rz(max_step_angle_ * cmd.angular().z());
      }

      step.footId() = ow::FootId::LEFT;
    }

    // Compute absolute step pose
    // T_next_w = T_prev_w * T_next_prev
    step.pos() = prev_step.pos() * X_rel;
    step.finalStep() = false;
    step.nStep() = prev_step.nStep() + 1;

    return step;
  }

  ow::FootStep SingleStepPlanner::planFinalStep(const ow::FootStep &prev_step)
  {
    ow::FootStep step;
    
    ow::CartesianPosition X_rel = ow::CartesianPosition::Zero();
    if (prev_step.footId() == ow::FootId::LEFT)
    {
      // Right foot step
      X_rel.linear().y() = -feet_separation_;
      step.footId() = ow::FootId::RIGHT;
    }
    else
    {
      // Left foot step
      X_rel.linear().y() = feet_separation_;
      step.footId() = ow::FootId::LEFT;
    }

    // Compute absolute step pose
    // T_next_w = T_prev_w * T_next_prev
    step.nStep() = prev_step.nStep() + 1;
    step.pos() = prev_step.pos() * X_rel;
    step.finalStep() = true;
    return step;
  }

} // namespace ow_fs_planner
