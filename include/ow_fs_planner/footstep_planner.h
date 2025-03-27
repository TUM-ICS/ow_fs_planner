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

#ifndef OPEN_WALKER_FOOTSTEP_PLANNER_H
#define OPEN_WALKER_FOOTSTEP_PLANNER_H

#include <ow_core/interfaces/i_foot_step_planner.h>
#include <ow_core/math.h>

#include <ow_fs_planner/single_step_planner.h>

/*!
 * \brief Open Walker footstep planner module namespace. These classes implement
 * the footstep planning alogirthms.
 */
namespace ow_fs_planner
{

/*!
 * \brief The FootstepPlanner class
 *
 * This class implements a basic footstep planner which uses only kinematic
 * parameters.
 *
 */
class FootstepPlanner :
    public ow::IFootStepPlanner
{
public:
  typedef ow::IFootStepPlanner Base;

protected:
  ow::Parameter parameter_;

  SingleStepPlanner single_planner_;

  ow::FootStepList plan_;               //!< FootStep plan.
  size_t cur_step_id_;                  //!< Points to current step in the Plan

  bool    fixed_plan_;                  //!< fixed plan flag.
  bool    initialized_;                 //!< Initialization flag.

  ow::Scalar  max_fwd_step_size_;       //!< Max forward step size.
  ow::Scalar  max_bwd_step_size_;       //!< Max Backwards step size.
  ow::Scalar  max_side_step_size_;      //!< Max lateral speed.
  ow::Scalar  max_step_angle_;          //!< Max anglular speed.
  ow::Scalar  feet_separation_;         //!< Max anglular speed.

  size_t  number_of_steps_;        //!< number of steps to be executed.
  ow::Scalar  fwd_step_size_cmd_;  //!< Fixed plan step size command.
  ow::Scalar  lateral_disp_cmd_;   //!< Fixed plan lateral displacement command.
  ow::Scalar  step_angle_cmd_;     //!< Fixed plan angular command.
  ow::CartesianVelocity cmd_;      //!< Adapted velocity command.

public:

  /*!
   * \brief Default constructor
   */
  FootstepPlanner();

  virtual ~FootstepPlanner();

  /*!
   * \brief Initialize foot step planner.
   *
   * \param T_l_w
   *    Initial Left Foot pose wrt world.
   *
   * \param T_r_w
   *    Initial Right Foot pose wrt world.
   */
  const ow::FootStepList& start( const ow::CartesianPosition& X_l_w,
                                 const ow::CartesianPosition& X_r_w );

  /**
   * @brief Reset the planner to left and right step in foot_steps.
   * The next plan that is generated will start from the given foot_steps.
   * 
   * @param inital_foot_steps 
   */
  const ow::FootStepList& reset( const ow::FootStepList& foot_steps);

  void abort(const ow::FootStep& current_step);

  /*!
   * \brief Update footstep plan from current step landing. When the current
   *    step is different than the expected step, the following steps shall be
   *    adjusted according to the difference. This function should be called as
   *    soon as a foot is considered landed.
   *
   * \param current_step
   *    New current step.
   *
   * \return
   *    0: when the plan was succesfully updated.
   *    -1: if the received fooftsep's refference foot is different than the
   *    expected root reference (in the previously planned steps).
   *    -2: if the number of step id of the received step is larger than the
   *    size of the footstep plan.
   *    -3: if the module has not been initialized yet.
   */
  int update(const ow::FootStep& current_step);

  /*!
   * \brief Generate a fixed plan from standing position. This is a useful
   *    initial motion command for repeatable walking tests.
   *
   * \param fwd_step_size
   *    Forward (or backwards) step size.
   *
   * \param lateral_disp
   *    Lateral displacement for side stepping.
   *
   * \param step_angle
   *    Angular displacement for turning steps.
   *
   * \param number_of_steps
   *    Number of steps to execute with the former parameters.
   *
   * \return Generated footstep plan.
   */
  const ow::FootStepList& generateFixedPlan(
      ow::Scalar fwd_step_size,
      ow::Scalar lateral_disp,
      ow::Scalar step_angle,
      size_t number_of_steps);

  const ow::FootStepList& generateFixedPlan();

  /*!
   * \brief Generate footstep plan from velocity command. This command generates
   *    four step ahead. Just the necessary stepr to plan the DCM waypoints.
   *
   * \param velCmd
   *    Velocity command wrt base link.
   *
   * \return Generated footstep plan.
   */
  const ow::FootStepList& planSteps(
      const ow::FootStep& current_step,
      const ow::CartesianVelocity& velCmd);

  /*!
   * \brief Plan a series of steps from kinematic parameters.
   *
   * \param fwd_step_size
   *    Forward (or backwards) step size.
   *
   * \param lateral_disp
   *    Lateral displacement for side stepping.
   *
   * \param step_angle
   *    Angular displacement for turning steps.
   *
   * \return
   *    Vector containing the footstep plan.
   */
  const ow::FootStepList& planSteps(
      const ow::FootStep& current_step,
      const double fwd_step_size = 0.2,
      const double lateral_disp = 0.1,
      const double step_angle = 0.35);

  /*!
   * \brief Compute direction command for footstep planner from user defined
   *    kinematic parameters.
   *
   * \param fwd_step_size
   *    Forward (or backwards) step size.
   *
   * \param lateral_disp
   *    Lateral displacement for side stepping.
   *
   * \param step_angle
   *    Angular displacement for turning steps.
   *
   * \return
   *    Direcction command for the footstep planner.
   */
  ow::CartesianVelocity computeCommand(
      double fwd_step_size = 0.2,
      double lateral_disp = 0.0,
      double step_angle = 0.0);

  /*!
   * \brief Printout the footstep plan using the ROS_INFO_STREAM macro. Use only
   *    for debuging.
   */
  void print();

  /*!
  * \brief Output port function.
  *
  * \return
  *     Planned footsteps.
  */
  virtual const ow::FootStepList& footSteps() const;

protected:
  /*!
  * \brief Initialization of Forward Kinematics module.
  *
  * \return
  *    true on success.
  */
  virtual bool init(const ow::Parameter& parameter, ros::NodeHandle& nh);

};

}

#endif // OPEN_WALKER_FOOTSTEP_PLANNER_H
