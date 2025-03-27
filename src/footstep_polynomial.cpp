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

#include <ow_fs_planner/footstep_polynomial.h>

namespace ow_fs_planner
{

  FootstepPolynomial::FootstepPolynomial() : Base("FootstepPolynomial"),
                                       max_fwd_step_size_(0.0),
                                       max_bwd_step_size_(0.0),
                                       max_side_step_size_(0.0),
                                       max_step_angle_(0.0),
                                       feet_separation_(0.0),
                                       number_of_steps_(0),
                                       fwd_step_size_cmd_(0.0),
                                       lateral_disp_cmd_(0.0),
                                       step_angle_cmd_(0.0)
  {
  }

  FootstepPolynomial::~FootstepPolynomial()
  {
  }

  bool FootstepPolynomial::init(const ow::Parameter &parameter, ros::NodeHandle &nh)
  {
    // build the configuration
    parameter_.add<ow::Scalar>("feet_separation", 0.15);
    parameter_.add<ow::Scalar>("max_fwd_step_size", 0.2);
    parameter_.add<ow::Scalar>("max_bwd_step_size", 0.1);
    parameter_.add<ow::Scalar>("max_side_step_size", 0.1);
    parameter_.add<ow::Scalar>("max_step_angle", 0.35);
    /*parameter_.add<ow::Scalar>("fixed_plan/fwd_step_size", 0.1);
    parameter_.add<ow::Scalar>("fixed_plan/side_step_size", 0.1);
    parameter_.add<ow::Scalar>("fixed_plan/step_angle", 0.35);*/
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
    /*parameter_.get("fixed_plan/fwd_step_size", fwd_step_size_cmd_);
    parameter_.get("fixed_plan/side_step_size", lateral_disp_cmd_);
    parameter_.get("fixed_plan/step_angle", step_angle_cmd_);*/
    fwd_step_size_cmd_ = 0.1;
    lateral_disp_cmd_ = 0.1;
    step_angle_cmd_ = 0.35;

    ow::HomogeneousTransformation T_l_w, T_r_w;
    T_l_w.setIdentity();
    T_r_w.setIdentity();
    T_l_w.pos().y() = feet_separation_ / 2.0;
    T_r_w.pos().y() = -feet_separation_ / 2.0;
    start(T_l_w, T_r_w);

    return true;
  }

  const ow::FootStepList& FootstepPolynomial::start(
      const ow::HomogeneousTransformation &T_l_w,
      const ow::HomogeneousTransformation &T_r_w)
  {
    ow::FootStepList foot_steps(2); 
    foot_steps[0].pos() = T_r_w;
    foot_steps[1].pos() = T_l_w;
    foot_steps[0].footId() = ow::FootId::RIGHT;
    foot_steps[1].footId() = ow::FootId::LEFT;
    return reset(foot_steps);
  }

  const ow::FootStepList& FootstepPolynomial::reset( const ow::FootStepList& foot_steps)
  {
    if(foot_steps.size() < 2)
    {
      ROS_ERROR_STREAM("Footstep planner reset requires last left and right step");
      return foot_steps;
    }

    // copy inital footsteps, reset the current step
    plan_ = foot_steps;
    cur_step_id_ = 1;

    return plan_;
  }

  int FootstepPolynomial::update(const ow::FootStep& current_step)
  {
    return 0;
  }

  const ow::FootStepList &FootstepPolynomial::generateFixedPlan(
      const ow::CartesianPosition& X_goal_w)
  {
    // compute the starting com position (I assume we are standing!)
    ow::CartesianPosition X_current_w;
    X_current_w.linear() = 0.5*(plan_[cur_step_id_].pos().linear() + plan_[cur_step_id_-1].pos().linear());
    X_current_w.angular() = plan_[cur_step_id_].pos().angular();

    // determine which mode to use
    ow::LinearPosition x_delta = X_goal_w.linear() - X_current_w.linear();

    if(x_delta.norm() < 0.1)
    {
      // basically no motion, just turning
      return planStepTurn(X_goal_w, X_current_w, plan_[cur_step_id_]);
    }
    else if(std::abs(x_delta.y()) > std::abs(x_delta.x())/2.)
    {
      // sidestepping seems to be best
      return planStepSideWays(X_goal_w, X_current_w, plan_[cur_step_id_]);
    }
    else
    {
      // move forward to the goal
      return planStepsForward(X_goal_w, X_current_w, plan_[cur_step_id_]);
    }
  }

  const ow::FootStepList& FootstepPolynomial::planStepsForward(
      const ow::CartesianPosition& X_goal_w,
      const ow::CartesianPosition& X_current_w,
      const ow::FootStep& current_step)
  {
    // follow the spline with forward steps

    // construct a polynomial to the goal
    ow::LinearStateTrajectory spline = Spline(X_current_w, X_goal_w);

    // length of the trajectory
    ow::Scalar lenght = spline.arcLength(0, spline.endTime());
    size_t n_steps = lenght / max_fwd_step_size_;

    ow::CartesianPosition X_foot_i = ow::CartesianPosition::Identity();

    for(size_t i = 1; i <= n_steps; ++i)
    {
      const ow::FootStep& prev_step = plan_.back();

      // compute the step on the middle line
      ow::Scalar t = spline.inverseArcLength(0, i*max_fwd_step_size_);

      // compute the frame at time t
      ow::CartesianPosition X_i_w = spline.frame2D(t);

      // create new footstep
      ow::FootStep step;

      if(prev_step.footId() == ow::FootId::LEFT)
      {
        X_foot_i.linear().y() = -0.5*feet_separation_;
        step.footId() = ow::FootId::RIGHT;
      }
      else
      {
        X_foot_i.linear().y() = 0.5*feet_separation_;
        step.footId() = ow::FootId::LEFT;
      }

      step.pos() = X_i_w*X_foot_i;
      step.finalStep() = false;
      step.nStep() = prev_step.nStep() + 1;
      plan_.push_back(step);
    }

    plan_.push_back(planFinalStep(plan_.back()));
    return plan_;
  }

  const ow::FootStepList& FootstepPolynomial::planStepTurn(
      const ow::CartesianPosition& X_goal_w,
      const ow::CartesianPosition& X_current_w,
      const ow::FootStep& current_step)
  {
    // turing in place without any rotation
    ow::CartesianPosition X_goal_current = X_current_w.inverse()*X_goal_w;
    ow::Scalar yaw = X_goal_current.angular().eulerYPR()[0];

    ow::CartesianPosition X_rel = ow::CartesianPosition::Identity();

    ow::Scalar n_steps = std::abs(yaw) / max_step_angle_;
    for(size_t i = 0; i < n_steps; ++i)
    {
      const ow::FootStep& prev_step = plan_.back();

      // create new footstep
      ow::FootStep step;

      if(prev_step.footId() == ow::FootId::LEFT)
      {
        X_rel.linear().y() = -feet_separation_;
        if (yaw < 0.0)
        {
          X_rel.angular() = ow::Rotation3::Rz(max_step_angle_*ow::sign(yaw));
        }
        step.footId() = ow::FootId::RIGHT;
      }
      else
      {
        // Inward turning ( < 0) is neglected.
        X_rel.linear().y() = feet_separation_;
        if (yaw > 0.0)
        {
          X_rel.angular() = ow::Rotation3::Rz(max_step_angle_*ow::sign(yaw));
        }
        step.footId() = ow::FootId::LEFT;
      }

      step.pos() = prev_step.pos() * X_rel;
      step.finalStep() = false;
      step.nStep() = prev_step.nStep() + 1;
      plan_.push_back(step);
    }

    plan_.push_back(planFinalStep(plan_.back()));
    print();
    return plan_;
  }

  const ow::FootStepList& FootstepPolynomial::planStepSideWays(
      const ow::CartesianPosition& X_goal_w,
      const ow::CartesianPosition& X_current_w,
      const ow::FootStep& current_step)
  {
    // follow the plan by moving sideways

    ow::CartesianPosition X_goal_current = X_current_w.inverse()*X_goal_w;
    ow::Scalar yaw_goal = X_goal_current.angular().eulerYPR()[0];

    int step_right;
    if(X_goal_current.linear().y() < 0.0)
    {
      setFirstFootStepId(ow::FootId::LEFT);
      step_right = -1.;
    }
    else
    {
      setFirstFootStepId(ow::FootId::RIGHT);
      step_right = 1.;
    }

    // construct a polynomial to the goal
    ow::LinearStateTrajectory spline = Spline(X_current_w, X_goal_w);

    // length of the trajectory
    ow::Scalar lenght = spline.arcLength(0, spline.endTime());
    size_t n_steps = lenght / max_fwd_step_size_;

    ow::CartesianPosition X_foot_i = ow::CartesianPosition::Identity();

    for(size_t i = 1; i <= n_steps; ++i)
    {
      const ow::FootStep& prev_step = plan_.back();

      // compute the step on the middle line
      ow::Scalar t = spline.inverseArcLength(0, i*max_fwd_step_size_);
      ow::Scalar yaw = t/spline.duration()*yaw_goal;

      // compute the frame at time t
      ow::CartesianPosition X_i_w = spline.frame2D(t);

      // create new footstep
      ow::FootStep step;
      ow::FootStep next_step;

      // step to the side
      X_foot_i.linear().y() = step_right*max_side_step_size_;

      // leading foot pose moves along the spline
      step.pos().linear() = X_i_w.linear() + X_foot_i.linear();
      step.pos().angular() = X_current_w.angular()*ow::Rotation3::Rz(yaw);

      // follow foot pose moves relative to leading foot
      X_foot_i.linear().y() = -step_right*feet_separation_;
      next_step.pos() = step.pos()*X_foot_i;

      if(prev_step.footId() == ow::FootId::LEFT)
      {
        step.footId() = ow::FootId::RIGHT;
        next_step.footId() = ow::FootId::LEFT;
      }
      else
      {
        step.footId() = ow::FootId::LEFT;
        next_step.footId() = ow::FootId::RIGHT;
      }

      step.finalStep() = false;
      next_step.finalStep() = false;

      step.nStep() = prev_step.nStep() + 1;
      next_step.nStep() = step.nStep() + 1;

      plan_.push_back(step);
      plan_.push_back(next_step);
    }

    plan_.push_back(planFinalStep(plan_.back()));
    return plan_;
  }

  ow::FootStep FootstepPolynomial::planFinalStep(const ow::FootStep &prev_step)
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

  bool FootstepPolynomial::setFirstFootStepId(ow::FootId id)
  {
    if(plan_.size() < 2)
    {
      // something is wrong
      return false;
    }
    
    if(plan_[1].footId() == id)
    {
      // we are done
      return true;
    }

    // we need to swap things
    ow::FootStep tmp = plan_[0];
    plan_[0] = plan_[1];
    plan_[1] = tmp;
    plan_[0].nStep() = tmp.nStep();
    plan_[1].nStep() = tmp.nStep()++;
    return true;
  }

  void FootstepPolynomial::print()
  {
    for (unsigned int i = 0; i < plan_.size(); i++)
    {
      ROS_INFO_STREAM(" ");
      ROS_INFO_STREAM("Step " << i << ": ");
      ROS_INFO_STREAM("nStep: " << plan_.at(i).nStep());
      ROS_INFO_STREAM("Pos: " << plan_.at(i).pos().linear().transpose());
      ROS_INFO_STREAM("Leg: " << plan_.at(i).footId().toString());
      ROS_INFO_STREAM("stop: " << plan_.at(i).finalStep());
    }
  }

  const ow::FootStepList &FootstepPolynomial::footSteps() const
  {
    return plan_;
  }

} // namespace ow_fs_planner
