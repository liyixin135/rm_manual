//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_shooter_cover_manual.h"

namespace rm_manual
{
ChassisGimbalShooterCoverManual::ChassisGimbalShooterCoverManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : ChassisGimbalShooterManual(nh, nh_referee)
{
  ros::NodeHandle cover_nh(nh, "cover");
  nh.param("supply_frame", supply_frame_, std::string("supply_frame"));
  cover_command_sender_ = new rm_common::JointPositionBinaryCommandSender(cover_nh);
  ros::NodeHandle buff_switch_nh(nh, "buff_switch");
  switch_buff_srv_ = new rm_common::SwitchDetectionCaller(buff_switch_nh);
  ros::NodeHandle buff_type_switch_nh(nh, "buff_type_switch");
  switch_buff_type_srv_ = new rm_common::SwitchDetectionCaller(buff_type_switch_nh);
  ros::NodeHandle chassis_nh(nh, "chassis");
  normal_speed_scale_ = chassis_nh.param("normal_speed_scale", 1);
  low_speed_scale_ = chassis_nh.param("low_speed_scale", 0.30);
  nh.param("exit_buff_mode_duration", exit_buff_mode_duration_, 0.5);

  ctrl_z_event_.setEdge(boost::bind(&ChassisGimbalShooterCoverManual::ctrlZPress, this),
                        boost::bind(&ChassisGimbalShooterCoverManual::ctrlZRelease, this));
  z_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterCoverManual::zPressing, this));
  z_event_.setFalling(boost::bind(&ChassisGimbalShooterCoverManual::zRelease, this));
}

void ChassisGimbalShooterCoverManual::changeSpeedMode(SpeedMode speed_mode)  // 给予速度模式
{
  if (speed_mode == LOW)
  {
    speed_change_scale_ = low_speed_scale_;
  }
  else if (speed_mode == NORMAL)
  {
    speed_change_scale_ = normal_speed_scale_;
  }
}

void ChassisGimbalShooterCoverManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)  // 更新dbus信息
{
  ChassisGimbalShooterManual::updatePc(dbus_data);
  gimbal_cmd_sender_->setRate(-dbus_data->m_x * gimbal_scale_,
                              cover_command_sender_->getState() ? 0.0 : dbus_data->m_y * gimbal_scale_);
}

void ChassisGimbalShooterCoverManual::checkReferee()  // 检查裁判系统
{
  manual_to_referee_pub_data_.cover_state = cover_command_sender_->getState();
  if (switch_detection_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    manual_to_referee_pub_data_.det_target = switch_buff_type_srv_->getTarget();
  else
    manual_to_referee_pub_data_.det_target = switch_detection_srv_->getTarget();
  ChassisGimbalShooterManual::checkReferee();
}
void ChassisGimbalShooterCoverManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)  // 检查键盘
{
  ChassisGimbalShooterManual::checkKeyboard(dbus_data);
  ctrl_z_event_.update(dbus_data->key_ctrl & dbus_data->key_z);
  z_event_.update((!dbus_data->key_ctrl) & dbus_data->key_z);
}

void ChassisGimbalShooterCoverManual::sendCommand(const ros::Time& time)  // 发送命令
{
  if (supply_)
  {
    chassis_cmd_sender_->getMsg()->follow_source_frame = supply_frame_;
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    cover_close_ = false;
    try
    {
      double roll, pitch, yaw;
      quatToRPY(tf_buffer_.lookupTransform("base_link", supply_frame_, ros::Time(0)).transform.rotation, roll, pitch,
                yaw);
      if (std::abs(yaw) < 0.05)
        cover_command_sender_->on();
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
  }
  else
  {
    cover_command_sender_->off();
    if (!cover_close_)
    {
      try
      {
        double roll, pitch, yaw;
        quatToRPY(tf_buffer_.lookupTransform("base_link", "cover", ros::Time(0)).transform.rotation, roll, pitch, yaw);
        if (pitch - cover_command_sender_->getMsg()->data > 0.05)
        {
          chassis_cmd_sender_->getMsg()->follow_source_frame = supply_frame_;
          chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
        }
        else
        {
          cover_close_ = true;
          chassis_cmd_sender_->getMsg()->follow_source_frame = "yaw";
        }
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
      }
    }
  }
  ChassisGimbalShooterManual::sendCommand(time);
  cover_command_sender_->sendCommand(time);
}

void ChassisGimbalShooterCoverManual::rightSwitchDownRise()
{
  ChassisGimbalShooterManual::rightSwitchDownRise();
  supply_ = true;
}

void ChassisGimbalShooterCoverManual::rightSwitchMidRise()
{
  ChassisGimbalShooterManual::rightSwitchMidRise();
  supply_ = false;
}

void ChassisGimbalShooterCoverManual::rightSwitchUpRise()
{
  ChassisGimbalShooterManual::rightSwitchUpRise();
  supply_ = false;
}

void ChassisGimbalShooterCoverManual::rPress()  // 按下r
{
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)  // 如果目标不是装甲板
  {
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);       // 底盘功率是charge
    if (switch_buff_type_srv_->getTarget() == rm_msgs::StatusChangeRequest::SMALL_BUFF)  // 如果目标是小幅
      switch_buff_type_srv_->setTargetType(rm_msgs::StatusChangeRequest::BIG_BUFF);      // 更改为打大幅
    else
      switch_buff_type_srv_->setTargetType(rm_msgs::StatusChangeRequest::SMALL_BUFF);  // 大幅变为小幅
    switch_buff_type_srv_->callService();                                              // call服务
  }
  else
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);  // 底盘功率是normal
}

void ChassisGimbalShooterCoverManual::ePress()  // 按下e
{
  switch_buff_srv_->switchTargetType();                                 // 更换target为相反，0改为1,1改为0
  switch_detection_srv_->switchTargetType();                            // 更换target为相反，0改为1,1改为0
  switch_buff_type_srv_->setTargetType(switch_buff_srv_->getTarget());  // return target
  switch_buff_srv_->callService();                                      // call服务
  switch_detection_srv_->callService();
  switch_buff_type_srv_->callService();
}

void ChassisGimbalShooterCoverManual::zPressing()
{
  shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::MINIMAL);
}

void ChassisGimbalShooterCoverManual::zRelease()
{
  shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::LOW);
}

void ChassisGimbalShooterCoverManual::wPress()
{
  ChassisGimbalShooterManual::wPress();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    last_switch_time_ = ros::Time::now();
}

void ChassisGimbalShooterCoverManual::wPressing()
{
  ChassisGimbalShooterManual::wPressing();
  if ((ros::Time::now() - last_switch_time_).toSec() > exit_buff_mode_duration_ &&
      switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
  {
    switch_buff_srv_->setTargetType(rm_msgs::StatusChangeRequest::ARMOR);
    switch_detection_srv_->setTargetType(rm_msgs::StatusChangeRequest::ARMOR);
    switch_buff_type_srv_->setTargetType(switch_buff_srv_->getTarget());
    switch_buff_srv_->callService();
    switch_detection_srv_->callService();
    switch_buff_type_srv_->callService();
  }
}

void ChassisGimbalShooterCoverManual::ctrlZPress()
{
  if (!cover_command_sender_->getState())
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  else
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  supply_ = !cover_command_sender_->getState();
  if (supply_)
  {
    changeSpeedMode(LOW);
  }
  else
  {
    changeSpeedMode(NORMAL);
  }
}

}  // namespace rm_manual
