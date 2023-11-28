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

#include <fetch_hardware_interface.h>

namespace fetch_control
{

FetchHardwareInterface::FetchHardwareInterface()
  : joint_mode_(0),
    loop_hz_(100)
{
  printf("%d\n", joint_mode_);
  ROS_INFO_STREAM_NAMED("hardware_interface","Running in hardware mode");
  arm_hw_.reset(new fetch_control::ArmHardwareInterface("fetch_arm",loop_hz_));

  // Set the joint mode interface data
  hardware_interface::JointCommandModes mode;
  switch (joint_mode_)
  {
    case static_cast<int>(hardware_interface::JointCommandModes::MODE_POSITION):
      mode = hardware_interface::JointCommandModes::MODE_POSITION;
      break;
    case static_cast<int>(hardware_interface::JointCommandModes::MODE_VELOCITY):
      mode = hardware_interface::JointCommandModes::MODE_VELOCITY;
      break;
    case static_cast<int>(hardware_interface::JointCommandModes::MODE_EFFORT):
      mode = hardware_interface::JointCommandModes::MODE_EFFORT;
      break;
  }
  jm_interface_.registerHandle(hardware_interface::JointModeHandle("joint_mode", &mode));

  // Start the shared joint state subscriber
  sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1,
                     &FetchHardwareInterface::stateCallback, this);

  // Wait for first state message to be recieved if we are not in simulation
  // Loop until we find a joint_state message from Fetch
  do
  {
    // Loop until we get our first joint_state message
    while(ros::ok() && state_msg_timestamp_.toSec() == 0)
    {
      ROS_INFO_STREAM_NAMED("hardware_interface","Waiting for first state message to be recieved");
      ros::spinOnce();
      ros::Duration(0.25).sleep();
    }
  } while (state_msg_->name.size() != NUM_FETCH_JOINTS);

  // Initialize arm
  arm_hw_->init(js_interface_, ej_interface_, vj_interface_, pj_interface_, &joint_mode_, state_msg_);

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&jm_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&vj_interface_);
  registerInterface(&pj_interface_);

  // Create the controller manager
  ROS_INFO_STREAM_NAMED("hardware_interface","Loading controller_manager");
  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

  ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
  non_realtime_loop_ = nh_.createTimer(update_freq, &FetchHardwareInterface::update, this);

  ROS_INFO_NAMED("hardware_interface", "Loaded fetch_hardware_interface.");
}

FetchHardwareInterface::~FetchHardwareInterface()
{
}

bool FetchHardwareInterface::stateExpired()
{
  // Check that we have a non-expired state message
  // \todo lower the expiration duration
  if( ros::Time::now() > state_msg_timestamp_ + ros::Duration(STATE_EXPIRED_TIMEOUT)) // check that the message timestamp is no older than 1 second
  {

    ROS_WARN_STREAM_THROTTLE_NAMED(1,"hardware_interface","State expired. Last recieved state " << (ros::Time::now() - state_msg_timestamp_).toSec() << " seconds ago." );
    return true;
  }
  return false;
}

void FetchHardwareInterface::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  // Check if this message has the correct number of joints
  if( msg->name.size() != NUM_FETCH_JOINTS )
  {
    return;
  }

  // Copy the latest message into a buffer
  state_msg_ = msg;
  state_msg_timestamp_ = ros::Time::now();
}

void FetchHardwareInterface::update(const ros::TimerEvent& e)
{
  // Check if state msg from Fetch is expired
  if(stateExpired() )
    return;

  elapsed_time_ = ros::Duration(e.current_real - e.last_real);

  // Input
  arm_hw_->read(state_msg_);

  // Control
  controller_manager_->update(ros::Time::now(), elapsed_time_);

  // Output
  arm_hw_->write(elapsed_time_);
}

} // namespace

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("hardware_interface","Starting hardware interface...");

  ros::init(argc, argv, "fetch_hardware_interface");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  fetch_control::FetchHardwareInterface fetch;

  spinner.start();

  ros::waitForShutdown();

  ROS_INFO_STREAM_NAMED("hardware_interface","Shutting down.");

  return 0;
}



