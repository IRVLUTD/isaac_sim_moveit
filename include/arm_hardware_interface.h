/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   ros_control hardware interface layer for Baxter
*/

#ifndef FETCH_CONTROL__ARM_HARDWARE_INTERFACE_
#define FETCH_CONTROL__ARM_HARDWARE_INTERFACE_

// ROS
#include <trajectory_msgs/JointTrajectory.h>

// Parent class
#include <arm_interface.h>

namespace fetch_control
{

static const double STATE_EXPIRED_TIMEOUT = 2.0;

class ArmHardwareInterface : public ArmInterface
{
private:

  // Publishers
  ros::Publisher pub_joint_command_;
  ros::Publisher pub_trajectory_command_;

  // Subscriber
  ros::Subscriber cuff_squeezed_sub_; // this is used to update the controllers when manual mode is started

  // Messages to send
  sensor_msgs::JointState output_msg_;
  trajectory_msgs::JointTrajectory trajectory_command_msg_;

  // Track button status
  bool cuff_squeezed_previous;

  // Convert a joint states message to our ids
  std::vector<int> joint_id_to_joint_states_id_;

public:

  /**
   * \brief Constructor/Descructor
   */
  ArmHardwareInterface(const std::string &arm_name, double loop_hz);
  ~ArmHardwareInterface();

  /**
   * \brief Initialice hardware interface
   * \return false if an error occurred during initialization
   */
  bool init(
    hardware_interface::JointStateInterface&    js_interface,
    hardware_interface::EffortJointInterface&   ej_interface,
    hardware_interface::VelocityJointInterface& vj_interface,
    hardware_interface::PositionJointInterface& pj_interface,
    int* joint_mode,
    sensor_msgs::JointStateConstPtr state_msg
  );

  /**
   * \brief Buffers joint state info from Baxter ROS topic
   * \param
   */
  void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

  /**
   * \brief Copy the joint state message into our hardware interface datastructures
   */
  void read( sensor_msgs::JointStateConstPtr &state_msg );

  /**
   * \brief Publish our hardware interface datastructures commands to Baxter hardware
   */
  void write(ros::Duration elapsed_time);

  /**
   * \brief This is called when Baxter is disabled, so that we can update the desired positions
   */
  void robotDisabledCallback();

  /**
   * \brief inform the trajectory controller to update its setpoint
   */
  void publishCurrentLocation();
};

} // namespace

#endif
