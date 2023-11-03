#include "gripper_test.hpp"

#define BAUDRATE      38400
#define DEBUG_MODE    false
#define DATA_BIT      8
#define STOP_BIT      1
#define PARITY_MODE   'N'
#define DURATION_MIN  10
#define DURATION_MAX  100000
#define LOOP_FREQ     50

// MODBUS Start Address for Programmer  1번째 주소 = 0x0001 //
#define HOLDING_START_ADDRESS 0x0000

namespace gripper_ui {

GripperNode::GripperNode(int argc, char **argv, std::shared_ptr<rclcpp::Node> node)
    : init_argc(argc), init_argv(argv), nh_(node) {
}

GripperNode::~GripperNode() {
    modbusRelease();
    rclcpp::shutdown();
    wait();
}

bool GripperNode::init(char *port_name, uint slave_address) {
    if (modbusInit(port_name, slave_address) == -1) {
        ROS_LOG_ERROR("GripperNode initiating failed [Modbus init]");
        return false;
    }

    driver_enable_server_ = nh_->create_service<grp_control_msg::srv::DriverEnable>("driver_enable",
                            std::bind(&GripperNode::driverEnableCallback, this,
                            std::placeholders::_1, std::placeholders::_2));
    grp_pos_cmd_server_   = nh_->create_service<grp_control_msg::srv::PosVelCurCtrl>("gripper_position_control",
                            std::bind(&GripperNode::posCmdCallback, this,
                            std::placeholders::_1, std::placeholders::_2));
    grp_vel_cmd_server_   = nh_->create_service<grp_control_msg::srv::PosVelCurCtrl>("gripper_velocity_control",
                            std::bind(&GripperNode::velCmdCallback, this,
                            std::placeholders::_1, std::placeholders::_2));
    grp_cur_cmd_server_   = nh_->create_service<grp_control_msg::srv::PosVelCurCtrl>("gripper_current_control",
                            std::bind(&GripperNode::curCmdCallback, this,
                            std::placeholders::_1, std::placeholders::_2));

    grp_command_cmd_server_ = nh_->create_service<grp_control_msg::srv::GripperCommand>("gripper_command_control",
                              std::bind(&GripperNode::driverCommandCallback, this,
                              std::placeholders::_1, std::placeholders::_2));

    grp_state_publisher_ = nh_->create_publisher<grp_control_msg::msg::GripperMsg>
                           ("gripper_states", 1000);

    ROS_LOG_INFO("GripperNode : init");

    start();
    return true;
}

int GripperNode::modbusInit(char *port_name, uint slave_address) {
    std::unique_lock<std::mutex> lg(mutex_com_);

    ctx_ = modbus_new_rtu(port_name, BAUDRATE, PARITY_MODE, DATA_BIT, STOP_BIT);
    modbus_rtu_set_serial_mode(ctx_, MODBUS_RTU_RS485);
    modbus_rtu_set_rts_delay(ctx_, 300);

    if (ctx_ == NULL) {
        fprintf(stderr, "Unable to create the libmodbus context\n");
        return -1;
    }

    if (modbus_set_slave(ctx_, slave_address) == -1) {
        fprintf(stderr, "server_id=%d Invalid slave ID: %s\n", 1, modbus_strerror(errno));
        modbus_free(ctx_);
        return -1;
    }

    modbus_set_debug(ctx_, DEBUG_MODE);

    if (modbus_connect(ctx_) == -1) {
        fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
        modbus_free(ctx_);
        return -1;
    }

    modbus_connect_state_ = true;

    ROS_LOG_INFO("Modbus initiated");

    return 0;
}

void GripperNode::modbusRelease() {
    modbus_close(ctx_);
    modbus_free(ctx_);

    modbus_connect_state_ = false;

    ROS_LOG_INFO("Modbus released");
}

int GripperNode::slaveChange(uint slave_address) {
    std::unique_lock<std::mutex> lg(mutex_com_);

    setReadMode(false);

    if (modbus_set_slave(ctx_, slave_address) == -1) {
        fprintf(stderr, "server_id=%d Invalid slave ID: %s\n", 1, modbus_strerror(errno));
        modbus_free(ctx_);
        return -1;
    }

    modbus_set_debug(ctx_, DEBUG_MODE);

    if (modbus_connect(ctx_) == -1) {
        fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
        modbus_free(ctx_);
        return -1;
    }

    usleep(100000);
    ROS_LOG_INFO("Modbus slave address changed to %d", slave_address);

    setReadMode(true);

    return 0;
}

bool GripperNode::driverEnableCallback(const std::shared_ptr<grp_control_msg::srv::DriverEnable::Request> req,
                                       std::shared_ptr<grp_control_msg::srv::DriverEnable::Response> res) {
    return res->successed = (req->enable ? motorEnable() : motorDisable());
}

bool GripperNode::posCmdCallback(const std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Request> req,
                                 std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Response> res) {
    return res->successed = motorPosCtrl(req->position, req->duration);
}

bool GripperNode::velCmdCallback(const std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Request> req,
                                 std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Response> res) {
    return res->successed = motorVelCtrl(req->velocity, req->duration);
}

bool GripperNode::curCmdCallback(const std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Request> req,
                                 std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Response> res) {
    return res->successed = motorCurCtrl(req->current, req->duration);
}

bool GripperNode::driverCommandCallback(const std::shared_ptr<grp_control_msg::srv::GripperCommand::Request> req,
                                        std::shared_ptr<grp_control_msg::srv::GripperCommand::Response> res) {
    switch ((GRP_COMMAND) req->command) {
        case GRP_COMMAND::MOTOR_ENABLE:
            motorEnable();
            break;

        case GRP_COMMAND::MOTOR_STOP:
            motorStop(0);
            break;

        case GRP_COMMAND::MOTOR_DISABLE:
            motorDisable();
            break;

        case GRP_COMMAND::MOTOR_POSITION_CONTROL:
            motorPosCtrl(req->value_1, req->value_2);
            break;

        case GRP_COMMAND::MOTOR_VELOCITY_CONTROL:
            motorVelCtrl(req->value_1, req->value_2);
            break;

        case GRP_COMMAND::MOTOR_CURRENT_CONTROL:
            motorCurCtrl(req->value_1, req->value_2);
            break;

        case GRP_COMMAND::CHANGE_MODBUS_ADDRESS:

            break;

        case GRP_COMMAND::GRIPPER_INITIALIZE:
            grpInit();
            break;

        case GRP_COMMAND::GRIPPER_OPEN:
            grpOpen();
            break;

        case GRP_COMMAND::GRIPPER_CLOSE:
            grpClose();
            break;

        case GRP_COMMAND::SET_FINGER_POSITION:
            setFingerPos(req->value_1);
            break;

        case GRP_COMMAND::VACUUM_GRIPPER_ON:
            vacuumGrpOn();
            break;

        case GRP_COMMAND::VACUUM_GRIPPER_OFF:
            vacuumGrpOff();
            break;

        case GRP_COMMAND::SET_MOTOR_TORQUE:

            break;

        case GRP_COMMAND::SET_MOTOR_SPEED:

            break;

        default:
            ROS_LOG_ERROR("");

    }

    return res->successed = true;
}

void GripperNode::setReadMode(bool change_to_read_mode) {
    std::unique_lock<std::mutex> lg_var(mutex_var_);
    read_mode_ = change_to_read_mode;
}

bool GripperNode::motorEnable() {
    if (!modbus_connect_state_) {
        ROS_LOG_ERROR("Modbus communication is not enabled.");
        return false;
    }

    std::vector<uint16_t> buf_register(1);
    buf_register[0] = (uint16_t) GRP_COMMAND::MOTOR_ENABLE;

    return sendOrder("[Motor Enable]", buf_register);
}

bool GripperNode::motorDisable() {
    if (!modbus_connect_state_) {
        ROS_LOG_ERROR("Modbus communication is not enabled.");
        return false;
    }

    std::vector<uint16_t> buf_register(1);
    buf_register[0] = (uint16_t) GRP_COMMAND::MOTOR_DISABLE;

    return sendOrder("[Motor Disable]", buf_register);
}

// bool GripperNode::setParam(OtherParam other_param) {
//     setReadMode(false);

//     std::string error_prefix = "[Setting parameter]";

//     std::vector<uint16_t> buf_register(3);
//     MB_CMD_1 = MB_SET_MAX_VAL;
//     MB_CMD_2 = MB_2_NONE;

//     buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;
//     buf_register[1] = other_param.acc_max;
//     buf_register[2] = other_param.vel_max;

//     return sendOrder(error_prefix, buf_register);
// }

bool GripperNode::motorPosCtrl(int q_target, uint duration) {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Motor Position Control]";

    if (duration < DURATION_MIN) {
        ROS_LOG_INFO("%s Duration is too short ( < %dms)", error_prefix.c_str(), DURATION_MIN);
        return false;
    } else if (duration > DURATION_MAX) {
        ROS_LOG_INFO("%s Duration is too long ( > %dms)", error_prefix.c_str(), DURATION_MAX);
        return false;
    }

    std::vector<uint16_t> buf_register(3);

    buf_register[0] = (uint16_t) GRP_COMMAND::MOTOR_POSITION_CONTROL;
    buf_register[1] = (uint16_t) q_target;
    buf_register[2] = (uint16_t) duration;

    return sendOrder(error_prefix, buf_register);
}

bool GripperNode::motorVelCtrl(int qd_target, uint duration) {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Motor Velocity Control]";

    if (duration < DURATION_MIN) {
        ROS_LOG_INFO("%s Duration is too short ( < %dms)", error_prefix.c_str(), DURATION_MIN);
        return false;
    } else if (duration > DURATION_MAX) {
        ROS_LOG_INFO("%s Duration is too long ( > %dms)", error_prefix.c_str(), DURATION_MAX);
        return false;
    }

    std::vector<uint16_t> buf_register(3);

    buf_register[0] = (uint16_t) GRP_COMMAND::MOTOR_VELOCITY_CONTROL;
    buf_register[1] = (uint16_t) qd_target; // RPM
    buf_register[2] = (uint16_t) duration;  // ms

    return sendOrder(error_prefix, buf_register);
}

bool GripperNode::motorCurCtrl(int cur_target, uint duration) {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Motor Current Control]";

    if (duration < DURATION_MIN) {
        ROS_LOG_INFO("%s Duration is too short ( < %dms)", error_prefix.c_str(), DURATION_MIN);
        return false;
    } else if (duration > DURATION_MAX) {
        ROS_LOG_INFO("%s Duration is too long ( > %dms)", error_prefix.c_str(), DURATION_MAX);
        return false;
    }

    std::vector<uint16_t> buf_register(3);

    // Current(Amp)  = [Current(s16A) * Vdd micro] / [65536 * Rshunt * Aop]
    // Current(s16A) = [Current(Amp)  / Vdd micro] * [65536 * Rshunt * Aop]
    // Vdd : 3.3V
    // Aop : 4.375
    // Rshunt : 0.1ohm
    buf_register[0] = (uint16_t) GRP_COMMAND::MOTOR_CURRENT_CONTROL;
    buf_register[1] = (uint16_t) cur_target;
    buf_register[2] = (uint16_t) duration;

    return sendOrder(error_prefix, buf_register);
}

bool GripperNode::grpInit() {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Gripper Initialize]";
    std::vector<uint16_t> buf_register(1);

    buf_register[0] = (uint16_t) GRP_COMMAND::GRIPPER_INITIALIZE;

    return sendOrder(error_prefix, buf_register);
}

bool GripperNode::grpOpen() {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Gripper Open]";
    std::vector<uint16_t> buf_register(1);

    buf_register[0] = (uint16_t) GRP_COMMAND::GRIPPER_OPEN;

    return sendOrder(error_prefix, buf_register);
}

bool GripperNode::grpClose() {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Gripper Close]";
    std::vector<uint16_t> buf_register(1);

    buf_register[0] = (uint16_t) GRP_COMMAND::GRIPPER_CLOSE;

    return sendOrder(error_prefix, buf_register);
}

bool GripperNode::vacuumGrpOn() {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Vacuum Gripper On]";
    std::vector<uint16_t> buf_register(1);

    buf_register[0] = (uint16_t) GRP_COMMAND::VACUUM_GRIPPER_ON;

    return sendOrder(error_prefix, buf_register);
}

bool GripperNode::vacuumGrpOff() {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Vacuum Gripper Off]";
    std::vector<uint16_t> buf_register(1);

    buf_register[0] = (uint16_t) GRP_COMMAND::VACUUM_GRIPPER_OFF;

    return sendOrder(error_prefix, buf_register);
}

bool GripperNode::motorStop(uint16_t duration) {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Motor Stop]";
    std::vector<uint16_t> buf_register(1);

    buf_register[0] = (uint16_t) GRP_COMMAND::MOTOR_STOP;

    return sendOrder(error_prefix, buf_register);
}

bool GripperNode::setFingerPos(uint16_t Parameter) {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Set Finger Position]";
    std::vector<uint16_t> buf_register(2);

    buf_register[0] = (uint16_t) GRP_COMMAND::SET_FINGER_POSITION;
    buf_register[1] = Parameter;

    return sendOrder(error_prefix, buf_register);
}

bool GripperNode::sendOrder(std::string error_prefix, std::vector<uint16_t> buf_register) {
    printf("%s Command sent. \n", error_prefix.c_str());

    uint16_t register_number = buf_register.size();

    if (register_number == 1) {
        if (modbus_write_register(ctx_, HOLDING_START_ADDRESS, buf_register[0]) == -1) {
            fprintf(stderr, "%s Failed to modbus write register %d : %s\n", error_prefix.c_str(), HOLDING_START_ADDRESS, modbus_strerror(errno));
            return false;
        }
    } else if (modbus_write_registers(ctx_, HOLDING_START_ADDRESS, register_number, &buf_register[0]) == -1) {
        fprintf(stderr, "%s Failed to modbus write register %d : %s\n", error_prefix.c_str(), HOLDING_START_ADDRESS, modbus_strerror(errno));
        return false;
    }

    setReadMode(true);
    return true;
}

bool GripperNode::sendOrder_mutex(std::string error_prefix, std::vector<uint16_t> buf_register) {
    std::unique_lock<std::mutex> lg(mutex_com_);
    return sendOrder(error_prefix, buf_register);
}

bool GripperNode::checkValue() {
    static std::map<uint, std::vector<std::string>> status_info;

    if (status_info.size() == 0) {
        status_info.insert({0, {"false", "Motor Enable"}});
        status_info.insert({1, {"false", "Gripper Initialize"}});
        status_info.insert({2, {"false", "Motor Position Control"}});
        status_info.insert({3, {"false", "Motor Velocity Control"}});
        status_info.insert({4, {"false", "Motor Current Control"}});
        status_info.insert({5, {"false", "Gripper Open"}});
        status_info.insert({6, {"false", "Gripper Close"}});
        status_info.insert({9, {"false", "Motor Fault"}});
    }

    std::unique_lock<std::mutex> lg(mutex_com_);

    // Read input register //
    uint16_t reg_address = 10;
    uint16_t reg_num     = 8;
    uint16_t reg[reg_num] = {0, };

    if (modbus_read_registers(ctx_, reg_address, reg_num, reg) == -1) {
        fprintf(stderr, "Failed to read input registers! : %s\n", modbus_strerror(errno));
    }

    uint16_t status          = reg[0];
    int16_t  motor_position  = reg[1];
    int16_t  motor_current   = reg[2];
    int16_t  motor_velocity  = reg[3];
    uint16_t finger_position = reg[4];
    uint16_t bus_voltage     = reg[7];

    status_str_.clear();

    for (int i = 0; i < 16; i++) {
        if (status_info.find(i) != status_info.end()) {
            if (status & (0x01 << i)) {
                status_info[i][0] = "true";
                status_str_ = status_info[i][1];
            }
        }
    }

    if (status_str_.size() == 0) {
        status_str_ = "Motor Disabled";
    }

    msg_.motor_position  = motor_position;
    msg_.motor_velocity  = motor_velocity;
    msg_.motor_current   = motor_current;
    msg_.finger_position = finger_position;

    msg_.motor_enabled       = status_info[0][1] == "true";
    msg_.gripper_initialized = status_info[1][1] == "true";
    msg_.position_ctrl_mode  = status_info[2][1] == "true";
    msg_.velocity_ctrl_mode  = status_info[3][1] == "true";
    msg_.current_ctrl_mode   = status_info[4][1] == "true";
    msg_.grp_opened          = status_info[5][1] == "true";
    msg_.grp_closed          = status_info[6][1] == "true";
    msg_.motor_fault         = status_info[9][1] == "true";

    grp_state_publisher_->publish(msg_);
    return true;
}

// Main loop
void GripperNode::run() {
    rclcpp::Rate loop_rate(LOOP_FREQ);

    while(rclcpp::ok()) {
        rclcpp::spin_some(nh_);

        if (ctx_ != NULL && modbus_connect_state_ ) {
            std::unique_lock<std::mutex> lg_var(mutex_var_);

            if (!read_mode_) {
                lg_var.unlock();
                continue;
            }

            lg_var.unlock();
            checkValue();
        }

        loop_rate.sleep();
    }

    motorDisable();
    modbusRelease();

    Q_EMIT rclcpp::shutdown();
}

}   // end of namespace

