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

#include "grp_control_msg/msg/gripper_msg.hpp"

#include "grp_control_msg/srv/pos_vel_cur_ctrl.hpp"
#include "grp_control_msg/srv/driver_enable.hpp"
#include "grp_control_msg/srv/gripper_command.hpp"

#include "parameter_define.hpp"

#define NUM_REG 3

#define COIL_ENABLE    0
#define COIL_DISABLE   1
#define COIL_MOVE      2
#define COIL_POS_CTRL  3
#define COIL_VEL_CTRL  4
#define COIL_CUR_CTRL  5
#define COIL_SET_VALUE 6
#define COIL_STOP      7
#define COIL_INIT      8
#define COIL_OPEN      9
#define COIL_CLOSE     10
#define COIL_PARGRIP   11

#define MB_IDLE_STATE         0x0000
#define MB_ENABLE_STATE       0x0001
#define MB_GRP_INIT_STATE     0x0002
#define MB_POS_CTRL_STATE     0x0004
#define MB_VEL_CTRL_STATE     0x0008
#define MB_TOR_CTRL_STATE     0x0010
#define MB_GRP_OPEN_STATE     0x0020
#define MB_GRP_CLOSE_STATE    0x0040
#define MB_GRP_DIR_REV_STATE  0x0080
#define MB_GRP_OBJ_GRASP      0x0100
#define MB_FAULT_OCCURED      0x0200

// ROS log macro
#define ROS_LOG_INFO(...)  RCLCPP_INFO (rclcpp::get_logger("Gripper Control"), __VA_ARGS__);
#define ROS_LOG_WARN(...)  RCLCPP_WARN (rclcpp::get_logger("Gripper Control"), __VA_ARGS__);
#define ROS_LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("Gripper Control"), __VA_ARGS__);

typedef enum {
    MB_NONE    = 0,
    MB_ENABLE  = 1,
    MB_STOP_P  = 2,
    MB_STOP_V  = 3,
    MB_DISABLE = 4,

    MB_POS_CTRL = 5,
    MB_VEL_CTRL = 6,
    MB_TOR_CTRL = 7,

    MB_GRP_SET_DIR  = 100,
    MB_GRP_INIT     = 101,
    MB_GRP_OPEN     = 102,
    MB_GRP_CLOSE    = 103,
    MB_GRP_POS_CTRL = 104,
    MB_GRP_INIT2    = 105,
    MB_VAC_ON       = 106,
    MB_VAC_OFF      = 107,

    MB_SET_MAX_VAL = 201,

    MB_SET_PID_GAIN = 211,
} MB_CMD_1_t;

typedef enum {
    MB_2_NONE = 0,
} MB_CMD_2_t;


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

    bool driverEnable();
    bool driverDisable();
    bool grpPowerCtrl(bool tf);

    bool posCtrl(PosCtrlParam pos_ctrl_param);
    bool velCtrl(VelCtrlParam vel_ctrl_param);
    bool curCtrl(CurCtrlParam cur_ctrl_param);
    bool grpInit();
    bool grpOpen();
    bool vacuumGrpOn();
    bool vacuumGrpOff();
    bool grpClose();
    bool grp3FparClose();
    bool grpPosCtrl(uint16_t);

    bool stopMotor(StopMotorParam stop_motor_param);

    bool setParam(OtherParam other_param);

    bool sendOrder_noMutex(std::string error_prefix, std::vector<uint16_t> buf_register);

    bool sendOrder(std::string error_prefix, std::vector<uint16_t> buf_register);
    //bool sendOrder(std::string error_prefix, std::vector<uint16_t> buf_register, int coil_address);
    //bool sendOrder(std::string error_prefix, int coil_address);
    void toggleCheckValue();

    grp_control_msg::msg::GripperMsg msg_;

    MB_CMD_1_t  MB_CMD_1;
    MB_CMD_2_t  MB_CMD_2;

    std::string MB_STATUS;
    bool isMotorEnable;
    bool isGrpInitOngoing;
    bool isPosOngoing;
    bool isVelOngoing;
    bool isTorOngoing;
    bool isGrpOpening;
    bool isGrpClosing;
    bool grpDirection;
    bool isObjectGrasp;
    bool isFaultOccured;
    std::string MB_GRP_DIR;

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
