#ifndef GRIPPER_TEST_HPP
#define GRIPPER_TEST_HPP

#include <rclcpp/rclcpp.hpp>
#include <QThread>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <vector>
#include <math.h>
#include <mutex>
#include <modbus/modbus-rtu.h>
#include <map>

#include "grp_control_msg/msg/gripper_msg.hpp"

#include "grp_control_msg/srv/pos_vel_cur_ctrl.hpp"
#include "grp_control_msg/srv/driver_enable.hpp"
#include "grp_control_msg/srv/gripper_command.hpp"

#include "parameter_define.hpp"

// ROS log macro
#define ROS_LOG_INFO(...)  RCLCPP_INFO (rclcpp::get_logger("Gripper Control"), __VA_ARGS__);
#define ROS_LOG_WARN(...)  RCLCPP_WARN (rclcpp::get_logger("Gripper Control"), __VA_ARGS__);
#define ROS_LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("Gripper Control"), __VA_ARGS__);

enum class GRP_COMMAND {
    MOTOR_ENABLE           = 1,
    MOTOR_STOP             = 2,
    MOTOR_DISABLE          = 3,
    MOTOR_POSITION_CONTROL = 5,
    MOTOR_VELOCITY_CONTROL = 6,
    MOTOR_CURRENT_CONTROL  = 7,
    CHANGE_MODBUS_ADDRESS  = 50,
    GRIPPER_INITIALIZE     = 101,
    GRIPPER_OPEN           = 102,
    GRIPPER_CLOSE          = 103,
    SET_FINGER_POSITION    = 104,
    VACUUM_GRIPPER_ON      = 106,
    VACUUM_GRIPPER_OFF     = 107,
    SET_MOTOR_TORQUE       = 212,
    SET_MOTOR_SPEED        = 213,
};

struct DatcStatus {
    bool enable         = false;
    bool initialize     = false;
    bool motor_pos_ctrl = false;
    bool motor_vel_ctrl = false;
    bool motor_cur_ctrl = false;
    bool grp_open       = false;
    bool grp_close      = false;
    bool fault          = false;
};

namespace gripper_ui {

class GripperNode : public QThread {
    Q_OBJECT

public:
    GripperNode(int argc, char **argv, std::shared_ptr<rclcpp::Node> node);
    virtual ~GripperNode();

Q_SIGNALS:
    void rosShutdown();

public:
	bool init(char *port_name, uint slave_address);

    int modbusInit(char *port_name, uint slave_address);
    void modbusRelease();

    int slaveChange(uint slave_address);

    bool motorEnable();
    bool motorDisable();
    // bool setParam(OtherParam other_param);

    bool motorPosCtrl(int q_target  , uint duration);
    bool motorVelCtrl(int qd_target , uint duration);
    bool motorCurCtrl(int cur_target, uint duration);

    bool grpInit();
    bool grpOpen();
    bool grpClose();

    bool vacuumGrpOn();
    bool vacuumGrpOff();

    bool motorStop(uint16_t duration);

    bool setFingerPos(uint16_t);

    grp_control_msg::msg::GripperMsg msg_;

    DatcStatus datc_status_;
    std::string status_str_;

private:
	int init_argc;
	char **init_argv;

    std::shared_ptr<rclcpp::Node> nh_;

    rclcpp::Service<grp_control_msg::srv::DriverEnable>::SharedPtr driver_enable_server_;

    rclcpp::Service<grp_control_msg::srv::PosVelCurCtrl>::SharedPtr grp_pos_cmd_server_;
    rclcpp::Service<grp_control_msg::srv::PosVelCurCtrl>::SharedPtr grp_vel_cmd_server_;
    rclcpp::Service<grp_control_msg::srv::PosVelCurCtrl>::SharedPtr grp_cur_cmd_server_;

    rclcpp::Service<grp_control_msg::srv::GripperCommand>::SharedPtr grp_command_cmd_server_;
    rclcpp::Publisher<grp_control_msg::msg::GripperMsg>::SharedPtr grp_state_publisher_;

    modbus_t *ctx_;

    bool is_enable_            = false;
    bool modbus_connect_state_ = false;
    bool read_mode_            = true;

    std::mutex mutex_com_;
    std::mutex mutex_var_;

    bool sendOrder      (std::string error_prefix, std::vector<uint16_t> buf_register);
    bool sendOrder_mutex(std::string error_prefix, std::vector<uint16_t> buf_register);

	void run();
    bool driverEnableCallback(const std::shared_ptr<grp_control_msg::srv::DriverEnable::Request> req, std::shared_ptr<grp_control_msg::srv::DriverEnable::Response> res);

    bool posCmdCallback(const std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Request> req, std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Response> res);
    bool velCmdCallback(const std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Request> req, std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Response> res);
    bool curCmdCallback(const std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Request> req, std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Response> res);

    bool driverCommandCallback(const std::shared_ptr<grp_control_msg::srv::GripperCommand::Request> req, std::shared_ptr<grp_control_msg::srv::GripperCommand::Response> res);
    bool checkValue();

    void setReadMode(bool change_to_read_mode);
};

} // end of namespace

#endif // grp_control_msg_H
