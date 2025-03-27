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

#ifndef OPEN_WALKER_SINGLE_STEP_PLANNER_H
#define OPEN_WALKER_SINGLE_STEP_PLANNER_H

#include <ow_core/types.h>
#include <ow_core/common/parameter.h>

namespace ow_fs_planner
{

/*!
 * \brief The SingleStepPlanner class
 *
 * This class implements a basic footstep planner which uses only kinematic
 * parameters.
 */
class SingleStepPlanner
{
protected:
  double  max_fwd_step_size_;     //!< Max forward step size.
  double  max_bwd_step_size_;     //!< Max Backwards step size.
  double  max_side_step_size_;      //!< Max lateral displacement.
  double  max_step_angle_;        //!< Max angle for turning steps.
  double  feet_separation_;       //!< separation between the feet.

public:
  /*!
   * \brief Default constructor.
   */
  SingleStepPlanner();

  /**
   * @brief Initialize
   * 
   */
  bool init(const ow::Parameter& parameter);

  /*!
   * \brief Plan a step from the previous step and a direction command.
   *
   * \param prev_step
   *    Preavious step in the plan.
   *
   * \param cmd
   *    Direction command. It has to be in the range between -1 and 1 in X, Y,
   *    and Yaw (Z rotation) axis.
   *
   * \return
   *    Next footstep acording to step planner parameters and direction command.
   */
  ow::FootStep planStep(
    const ow::FootStep& prev_step, const ow::CartesianVelocity& cmd);

  /*!
   * \brief Generate the final step to come to a stop in standing position. The
   *    final step is marked as stopping step and the target pose will be just
   *    the feet separation.
   *
   * \param penult_step
   *    Penultimate step in the plan.
   *
   * \return
   *    Final footstep of the plan.
   */
  ow::FootStep planFinalStep(const ow::FootStep& penult_step);

};

}

#endif // OPEN_WALKER_SINGLE_STEP_PLANNER_H
