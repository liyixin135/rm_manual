//
// Created by peter on 2020/12/3.
//

#ifndef RM_MANUAL_MANUAL_BASE_H_
#define RM_MANUAL_MANUAL_BASE_H_

#include <iostream>
#include <queue>
#include <tf/transform_listener.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_loader.h>
#include <rm_common/decision/calibration_manager.h>
#include <controller_manager_msgs/SwitchController.h>

#include "rm_manual/common/data.h"
#include "rm_manual/common/input_event.h"
#include "rm_manual/referee/ui.h"

namespace rm_manual {

class ManualBase {
 public:
  explicit ManualBase(ros::NodeHandle &nh);
  ~ManualBase() {
    delete controller_loader_;
    delete calibration_manager_;
  }
  enum { PASSIVE, IDLE, RC, PC };
  virtual void run();
 protected:
  void checkReferee(const ros::Time &time);
  void checkSwitch(const ros::Time &time);
  virtual void checkKeyboard();
  virtual void updateRc();
  virtual void updatePc();
  virtual void sendCommand(const ros::Time &time) = 0;
  virtual void drawUi() {};

  // Referee
  virtual void chassisOutputOn() {};
  virtual void gimbalOutputOn() {};
  virtual void shooterOutputOn() {};

  // Remote Controller
  virtual void remoteControlTurnOff() {
    switch_base_ctrl_srv_->flipControllers();
    switch_base_ctrl_srv_->callService();
    state_ = PASSIVE;
  }
  virtual void remoteControlTurnOn() {
    switch_base_ctrl_srv_->switchControllers();
    switch_base_ctrl_srv_->callService();
    state_ = IDLE;
    calibration_manager_->reset();
  }
  virtual void leftSwitchDown(ros::Duration duration) {};
  virtual void leftSwitchMid(ros::Duration duration) {};
  virtual void leftSwitchUp(ros::Duration duration) {};
  virtual void rightSwitchDown(ros::Duration duration) { state_ = IDLE; }
  virtual void rightSwitchMid(ros::Duration duration) { state_ = RC; }
  virtual void rightSwitchUp(ros::Duration duration) { state_ = PC; }

  // Keyboard
  virtual void wPress(ros::Duration duration) {};
  virtual void wRelease(ros::Duration duration) {};
  virtual void sPress(ros::Duration duration) {};
  virtual void sRelease(ros::Duration duration) {};
  virtual void aPress(ros::Duration duration) {};
  virtual void aRelease(ros::Duration duration) {};
  virtual void dPress(ros::Duration duration) {};
  virtual void dRelease(ros::Duration duration) {};
  virtual void mouseLeftPress(ros::Duration duration) {};
  virtual void mouseLeftRelease(ros::Duration duration) {};
  virtual void mouseRightPress(ros::Duration duration) {};
  virtual void mouseRightRelease(ros::Duration duration) {};
  virtual void xPress(ros::Duration duration) {};
  virtual void xRelease(ros::Duration duration) {};
  virtual void ePress(ros::Duration duration) {};
  virtual void gPress(ros::Duration duration) {};

  Data data_;
  rm_common::ControllerLoader *controller_loader_;
  rm_common::CalibrationManager *calibration_manager_;
  rm_common::SwitchControllersService *switch_state_ctrl_srv_{}, *switch_base_ctrl_srv_{};

  bool remote_is_open_{};
  ros::NodeHandle nh_;
  int state_ = PASSIVE;
  RisingInputEvent w_press_event_;
  FallingInputEvent w_release_event_;
  RisingInputEvent s_press_event_;
  FallingInputEvent s_release_event_;
  RisingInputEvent a_press_event_;
  FallingInputEvent a_release_event_;
  RisingInputEvent d_press_event_;
  FallingInputEvent d_release_event_;
  RisingInputEvent mouse_left_press_event_;
  FallingInputEvent mouse_left_release_event_;
  RisingInputEvent mouse_right_press_event_;
  FallingInputEvent mouse_right_release_event_;
  RisingInputEvent x_press_event_;
  FallingInputEvent x_release_event_;
  RisingInputEvent e_press_event_;
  RisingInputEvent g_press_event_;
  RisingInputEvent switch_right_down_event_;
  RisingInputEvent switch_right_mid_event_;
  RisingInputEvent switch_right_up_event_;
  RisingInputEvent switch_left_down_event_;
  RisingInputEvent switch_left_mid_event_;
  RisingInputEvent switch_left_up_event_;
};

}
#endif // RM_MANUAL_MANUAL_BASE_H_
