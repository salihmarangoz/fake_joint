/**
 * @file fake_joint_driver.cpp
 * @author Ryosuke Tajima
 * @copyright 2016, 2017, Tokyo Opensource Robotics Kyokai Association
 * @license http://www.apache.org/licenses/LICENSE-2.0 Apache-2.0
 *
 * FakeJointDriver class (only do loopback from command to status)
 * derived from the hardware_interface class
 */
#include <ros/ros.h>
#include <urdf/model.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include "fake_joint_driver/fake_joint_driver.h"

FakeJointDriver::FakeJointDriver(void)
{
  ros::NodeHandle pnh("~");
  std::set<std::string> joint_set;
  std::map<std::string, double> start_position_map;

  // Read parameters
  pnh.param<bool>("use_robot_description", use_description_, true);
  pnh.getParam("include_joints", include_joints_);
  pnh.getParam("exclude_joints", exclude_joints_);
  pnh.getParam("start_position", start_position_map);
  pnh.param<double>("noise_std_dev", noise_std_dev_, 1e-4);
  pnh.param<double>("alpha_vel", alpha_vel_, 0.9);

  pos_noise_ = std::normal_distribution<double>(0.0, noise_std_dev_);

  for (auto it=start_position_map.begin(); it!=start_position_map.end(); it++)
  {
    ROS_DEBUG_STREAM("start_position: " << it->first << ": " << it->second);
  }

  for (auto i=0; i<include_joints_.size(); i++)
  {
    ROS_DEBUG_STREAM("include_joint[" << i << "]" << include_joints_[i]);
  }
  for (auto i=0; i<exclude_joints_.size(); i++)
  {
    ROS_DEBUG_STREAM("exclude_joint[" << i << "]" << exclude_joints_[i]);
  }
  // Read all joints in robot_description
  if (use_description_)
  {
    urdf::Model urdf_model;
    if (urdf_model.initParam("robot_description"))
    {
      for (auto it=urdf_model.joints_.begin(); it!=urdf_model.joints_.end(); it++)
      {
        urdf::Joint joint = *it->second;
        // remove fixed and unknown joints
        if (joint.type == urdf::Joint::FIXED || joint.type == urdf::Joint::UNKNOWN)
        {
          continue;
        }
        joint_set.insert(joint.name);
      }
    }
    else
    {
      ROS_WARN("We cannot find the parameter robot_description.");    
    }
  }
  // Include joints into joint_set
  for (auto i=0; i< include_joints_.size(); i++)
  {
    joint_set.insert(include_joints_[i]);
  }
  // Exclude joints in joint_set
  for (auto i=0; i< exclude_joints_.size(); i++)
  {
    joint_set.erase(exclude_joints_[i]);
  }
  // Convert to vector (joint_names_)
  std::copy(joint_set.begin(), joint_set.end(), std::back_inserter(joint_names_));
  // Check the emptyness of joints
  if (joint_names_.size() == 0) {
    ROS_ERROR("No joints is specified. Please use include_joints parameters.");
    ros::shutdown();
  }
  // Resize members
  cmd_dis.resize(joint_names_.size());
  act_dis.resize(joint_names_.size());
  act_vel.resize(joint_names_.size());
  act_eff.resize(joint_names_.size());

  //prev_vel_.resize(joint_names_.size());
  //prev_cmd_dis_.resize(joint_names_.size());

  // Set start position
  for (auto it=start_position_map.begin(); it!=start_position_map.end(); it++)
  {
    for (auto i=0; i<joint_names_.size(); i++)
    {
      if (joint_names_[i] == it->first)
      {
        act_dis[i] = it->second;
        cmd_dis[i] = it->second;
      }
    }
  }

  // Create joint_state_interface and position_joint_interface
  for (int i = 0; i< joint_names_.size(); i++)
  {
    ROS_DEBUG_STREAM("joint[" << i << "]:" << joint_names_[i]);
    // Connect and register the joint_state_interface
    hardware_interface::JointStateHandle state_handle(joint_names_[i], &act_dis[i], &act_vel[i], &act_eff[i]);
    joint_state_interface.registerHandle(state_handle);

    // Connect and register the position_joint_interface
    hardware_interface::JointHandle pos_handle(joint_state_interface.getHandle(joint_names_[i]), &cmd_dis[i]);
    position_joint_interface.registerHandle(pos_handle);
  }
  registerInterface(&joint_state_interface);
  registerInterface(&position_joint_interface);
}

FakeJointDriver::~FakeJointDriver()
{
}

/**
 * @brief Update function to call all of the update function of motors
 */
void FakeJointDriver::update(ros::Duration dt)
{ 
  // Backup previous pos
  std::vector<double> prev_pos = act_dis;

  for (int i=0; i<cmd_dis.size(); i++)
  {
    // Add white noise to the current pos
    act_dis[i] = cmd_dis[i] + pos_noise_(generator_);

    // Estimate current velocities via exponential weighted average
    act_vel[i] = (1-alpha_vel_)*act_vel[i] + alpha_vel_*(act_dis[i] - prev_pos[i]) / dt.toSec(); 
  }
  
  //prev_vel_ = act_vel;
  //prev_cmd_dis_ = cmd_dis;
}

