/**
 * @file fake_joint_driver.h
 * @author Ryosuke Tajima
 * @copyright 2016, 2017, Tokyo Opensource Robotics Kyokai Association
 * @license http://www.apache.org/licenses/LICENSE-2.0 Apache-2.0
 *
 * FakeJointDriver class (only do loopback from command to status)
 * derived from the hardware_interface class
 */
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <random>

class FakeJointDriver : public hardware_interface::RobotHW
{
private:
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface position_joint_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;

  std::vector<double> cmd_dis;
  std::vector<double> act_dis;
  std::vector<double> act_vel;
  std::vector<double> act_eff;

  std::vector<std::string> joint_names_;
  bool use_description_;
  std::vector<std::string> include_joints_;
  std::vector<std::string> exclude_joints_;
  std::default_random_engine generator_;
  std::normal_distribution<double> pos_noise_;
  double noise_std_dev_;
  double alpha_vel_;
  std::vector<double> prev_vel_, prev_cmd_dis_, prev_cmd_vel_, cmd_vel_;

public:
  FakeJointDriver(void);
  ~FakeJointDriver();
  void update(ros::Duration dt);
};
