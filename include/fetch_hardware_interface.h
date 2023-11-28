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

#ifndef FETCH_CONTROL__BAXTER_HARDWARE_INTERFACE_
#define FETCH_CONTROL__BAXTER_HARDWARE_INTERFACE_

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_mode_interface.h>
#include <hardware_interface/robot_hw.h>

// Fetch
#include <arm_interface.h>
#include <arm_hardware_interface.h>

namespace fetch_control
{

static const double NUM_FETCH_JOINTS = 15;

class FetchHardwareInterface : public hardware_interface::RobotHW
{
private:

  // Node Handles
  ros::NodeHandle nh_; // no namespace

  // Timing
  ros::Duration control_period_;
  ros::Time last_sim_time_ros_;
  ros::Duration elapsed_time_;
  double loop_hz_;

  // Interfaces
  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::JointModeInterface     jm_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  hardware_interface::PositionJointInterface pj_interface_;

  // sub-hardware interfaces
  fetch_control::ArmInterfacePtr arm_hw_;

  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  ros::Timer non_realtime_loop_;

  // Which joint mode are we in
  int joint_mode_;

  // Buffer of joint states to share between arms
  sensor_msgs::JointStateConstPtr state_msg_;
  ros::Time state_msg_timestamp_;

  // Subscriber
  ros::Subscriber sub_joint_state_;

public:

  /**
   * \brief Constructor/Descructor
   */
  FetchHardwareInterface();
  ~FetchHardwareInterface();

  /**
   * \brief Checks if the state message from Baxter is out of date
   * \return true if expired
   */
  bool stateExpired();

  void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

  void update(const ros::TimerEvent& e);

};

} // namespace

#endif
