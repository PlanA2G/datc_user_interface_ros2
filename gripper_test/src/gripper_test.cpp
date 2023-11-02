#include "gripper_test.hpp"

#define BAUDRATE      38400
#define DEBUG_MODE    false
#define DATA_BIT      8
#define STOP_BIT      1
#define PARITY_MODE   'N'
#define DURATION_MIN  10
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
    return res->successed = (req->enable ? driverEnable() : driverDisable());
}

bool GripperNode::posCmdCallback(const std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Request> req,
                                 std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Response> res) {
    PosCtrlParam pos_ctrl_param;

    pos_ctrl_param.pos      = req->position;
    pos_ctrl_param.vel      = req->velocity;
    pos_ctrl_param.duration = req->duration;

    return res->successed = posCtrl(pos_ctrl_param);
}

bool GripperNode::velCmdCallback(const std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Request> req,
                                 std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Response> res) {
    VelCtrlParam vel_ctrl_param;

    vel_ctrl_param.vel      = req->velocity;
    vel_ctrl_param.duration = req->duration;

    return res->successed = velCtrl(vel_ctrl_param);
}

bool GripperNode::curCmdCallback(const std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Request> req,
                                 std::shared_ptr<grp_control_msg::srv::PosVelCurCtrl::Response> res) {
    CurCtrlParam cur_ctrl_param;

    cur_ctrl_param.cur      = req->current;
    cur_ctrl_param.duration = req->duration;

    return res->successed = curCtrl(cur_ctrl_param);
}

bool GripperNode::driverCommandCallback(const std::shared_ptr<grp_control_msg::srv::GripperCommand::Request> req,
                                        std::shared_ptr<grp_control_msg::srv::GripperCommand::Response> res) {
    switch (req->command) {
        case MB_GRP_INIT:
            grpInit();
            break;

        case MB_GRP_OPEN:
            grpOpen();
            break;

        case MB_GRP_CLOSE:
            grpClose();
            break;

        case MB_GRP_POS_CTRL:
            grpPosCtrl(req->value);
            break;

        case MB_VAC_ON:
            vacuumGrpOn();
            break;

        case MB_VAC_OFF:
            vacuumGrpOff();
            break;
    }

    return res->successed = true;
}

bool GripperNode::driverEnable() {
    if (!modbus_connect_state_) {
        ROS_LOG_ERROR("Modbus is not initiated");
        return false;
    }

    std::vector<uint16_t> buf_register(1);
    MB_CMD_1 = MB_ENABLE;
    MB_CMD_2 = MB_2_NONE;

    buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;

    return sendOrder("[Enable]", buf_register);
}

bool GripperNode::driverDisable() {
    if (!modbus_connect_state_) {
        ROS_LOG_ERROR("Modbus is not initiated");
        return false;
    }

    std::vector<uint16_t> buf_register(1);
    MB_CMD_1 = MB_DISABLE;
    MB_CMD_2 = MB_2_NONE;

    buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;

    return sendOrder("[Disable]", buf_register);
}

void GripperNode::setReadMode(bool change_to_read_mode) {
    std::unique_lock<std::mutex> lg_var(mutex_var_);
    read_mode_ = change_to_read_mode;
}

bool GripperNode::setParam(OtherParam other_param) {
    setReadMode(false);

    std::string error_prefix = "[Setting parameter]";

    std::vector<uint16_t> buf_register(3);
    MB_CMD_1 = MB_SET_MAX_VAL;
    MB_CMD_2 = MB_2_NONE;

    buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;
    buf_register[1] = other_param.acc_max;
    buf_register[2] = other_param.vel_max;

    return sendOrder(error_prefix, buf_register);
}

bool GripperNode::posCtrl(PosCtrlParam pos_ctrl_param) {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);

    std::string error_prefix = "[Position Ctrl]";

    if (pos_ctrl_param.vel == 0 && pos_ctrl_param.duration < DURATION_MIN) {
        ROS_LOG_INFO("%s Duration is too low ( < %dms)", error_prefix.c_str(), DURATION_MIN);
        return false;
    }

    std::vector<uint16_t> buf_register(3);
    MB_CMD_1 = MB_POS_CTRL;
    MB_CMD_2 = MB_2_NONE;

    buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;
    buf_register[1] = (uint16_t)(pos_ctrl_param.pos);
    // buf_register[2] = pos_ctrl_param.duration != 0 ? pos_ctrl_param.duration : abs(pos_ctrl_param.pos) / abs(pos_ctrl_param.vel) * 1000;
    buf_register[2] = (uint16_t)(pos_ctrl_param.duration);

    return sendOrder_noMutex(error_prefix, buf_register);
}

bool GripperNode::velCtrl(VelCtrlParam vel_ctrl_param) {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);

    std::string error_prefix = "[Velocity Ctrl]";

    if (vel_ctrl_param.duration < DURATION_MIN) {
        ROS_LOG_INFO("%s Duration is too low ( < %dms)", error_prefix.c_str(), DURATION_MIN);
        return false;
    }

    std::vector<uint16_t> buf_register(3);
    MB_CMD_1 = MB_VEL_CTRL;
    MB_CMD_2 = MB_2_NONE;

    buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;
    buf_register[1] = (uint16_t)vel_ctrl_param.vel; // RPM
    buf_register[2] = vel_ctrl_param.duration;      // ms

    return sendOrder_noMutex(error_prefix, buf_register);
}

bool GripperNode::curCtrl(CurCtrlParam cur_ctrl_param) {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);

    std::string error_prefix = "[Current Ctrl]";

    if (cur_ctrl_param.duration < DURATION_MIN) {
        ROS_LOG_INFO("%s Duration is too low ( < %dms)", error_prefix.c_str(), DURATION_MIN);
        return false;
    }

    std::vector<uint16_t> buf_register(3);
    MB_CMD_1 = MB_VEL_CTRL;
    MB_CMD_2 = MB_2_NONE;

    // Current(Amp)  = [Current(s16A) * Vdd micro] / [65536 * Rshunt * Aop]
    // Current(s16A) = [Current(Amp)  / Vdd micro] * [65536 * Rshunt * Aop]
    // Vdd : 3.3V
    // Aop : 4.375
    // Rshunt : 0.1ohm
    buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;
    buf_register[1] = (uint16_t)cur_ctrl_param.cur;
    buf_register[2] = cur_ctrl_param.duration;

    return sendOrder_noMutex(error_prefix, buf_register);
}

bool GripperNode::grpInit() {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Gripper Init]";
    std::vector<uint16_t> buf_register(1);
    MB_CMD_1 = MB_GRP_INIT;
    MB_CMD_2 = MB_2_NONE;

    buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;

    return sendOrder_noMutex(error_prefix, buf_register);
}

bool GripperNode::grpOpen() {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Gripper Open]";
    std::vector<uint16_t> buf_register(1);
    MB_CMD_1 = MB_GRP_OPEN;
    MB_CMD_2 = MB_2_NONE;

    buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;

    return sendOrder_noMutex(error_prefix, buf_register);
}

bool GripperNode::grpClose() {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Gripper Close]";
    std::vector<uint16_t> buf_register(1);
    MB_CMD_1 = MB_GRP_CLOSE;
    MB_CMD_2 = MB_2_NONE;

    buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;

    return sendOrder_noMutex(error_prefix, buf_register);
}

bool GripperNode::vacuumGrpOn() {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Vacuum On]";
    std::vector<uint16_t> buf_register(1);
    MB_CMD_1 = MB_VAC_ON;
    MB_CMD_2 = MB_2_NONE;

    buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;

    return sendOrder_noMutex(error_prefix, buf_register);
}

bool GripperNode::vacuumGrpOff() {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Vacuum Off]";
    std::vector<uint16_t> buf_register(1);
    MB_CMD_1 = MB_VAC_OFF;
    MB_CMD_2 = MB_2_NONE;

    buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;

    return sendOrder_noMutex(error_prefix, buf_register);
}

bool GripperNode::stopMotor(StopMotorParam stop_motor_param) {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Stop motor]";
    std::vector<uint16_t> buf_register(1);

    MB_CMD_1 = MB_STOP_P;
    MB_CMD_2 = MB_2_NONE;

    buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;
    //&stop_motor_param.duration

    if (modbus_write_registers(ctx_, HOLDING_START_ADDRESS, 1, &buf_register[0]) == -1) {
        fprintf(stderr, "%s Failed to modbus_write_register_%d : %s\n", error_prefix.c_str(), 2, modbus_strerror(errno));
        return false;
    }

    setReadMode(true);

    return true;
}

bool GripperNode::grpPosCtrl(uint16_t Parameter) {
    setReadMode(false);

    std::unique_lock<std::mutex> lg(mutex_com_);
    std::string error_prefix = "[Gripper PosCtrl]";
    std::vector<uint16_t> buf_register(2);

    MB_CMD_1 = MB_GRP_POS_CTRL;
    MB_CMD_2 = MB_2_NONE;

    buf_register[0] = MB_CMD_2 << 8 | MB_CMD_1;
    buf_register[1] = Parameter;

    return sendOrder_noMutex(error_prefix, buf_register);
}

bool GripperNode::sendOrder_noMutex(std::string error_prefix, std::vector<uint16_t> buf_register) {
    printf("%s Command sent. \n", error_prefix.c_str());

    uint16_t register_number = buf_register.size();

    if (modbus_write_registers(ctx_, HOLDING_START_ADDRESS, register_number, &buf_register[0]) == -1) {
        fprintf(stderr, "%s Failed to modbus_write_register_%d : %s\n", error_prefix.c_str(), HOLDING_START_ADDRESS, modbus_strerror(errno));
        return false;
    }

    setReadMode(true);

    return true;
}

bool GripperNode::sendOrder(std::string error_prefix, std::vector<uint16_t> buf_register) {
    std::unique_lock<std::mutex> lg(mutex_com_);

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

    if (buf_register[0] == MB_ENABLE || buf_register[0] == MB_DISABLE) {
        is_enable_ = buf_register[0] == MB_ENABLE ? true : false;
        sleep(0.2);
    }

    setReadMode(true);

    return true;
}

bool GripperNode::checkValue() {
     std::unique_lock<std::mutex> lg(mutex_com_);

    // Read input register //
    uint16_t input_reg_address = 10;
    uint16_t input_reg_number  = 7;
    uint16_t input_reg[10] = {0, };

    if (modbus_read_registers(ctx_, input_reg_address, input_reg_number, input_reg) == -1) {
        fprintf(stderr, "Failed to read input registers! : %s\n", modbus_strerror(errno));
    }

    // MB_STATUS            = input_reg[0];
    uint16_t mb_status       = input_reg[0];
    uint16_t mb_position     = input_reg[1];
    uint16_t mb_torque       = input_reg[2];
    int16_t  mb_velocity     = input_reg[3];
    uint16_t mb_faultNow     = input_reg[4];
    uint16_t mb_faultOccured = input_reg[5];
    int16_t mb_value_1 = (int16_t)input_reg[4];

    //std::cout << mb_faultNow << "\t" << mb_faultOccured << std::endl;

    isMotorEnable     = (mb_status & (0x0001 <<  0)) != 0x0000; //0000 0000   0000 0001
    isGrpInitOngoing  = (mb_status & (0x0001 <<  1)) != 0x0000; //0000 0000   0000 0010
    isPosOngoing      = (mb_status & (0x0001 <<  2)) != 0x0000; //0000 0000   0000 0100
    isVelOngoing      = (mb_status & (0x0001 <<  3)) != 0x0000; //0000 0000   0000 1000
    isTorOngoing      = (mb_status & (0x0001 <<  4)) != 0x0000; //0000 0000   0001 0000
    isGrpOpening      = (mb_status & (0x0001 <<  5)) != 0x0000; //0000 0000   0010 0000
    isGrpClosing      = (mb_status & (0x0001 <<  6)) != 0x0000; //0000 0000   0100 0000
    grpDirection      = (mb_status & (0x0001 <<  7)) != 0x0000; //0000 0000   1000 0000
    isObjectGrasp     = (mb_status & (0x0001 <<  8)) != 0x0000; //0000 0001   0000 0000
    isFaultOccured    = (mb_status & (0x0001 <<  9)) != 0x0000; //0000 0010   0000 0000

    if (isMotorEnable) {
        /* Motor Status */
        if (isPosOngoing) {
            MB_STATUS = "Motor Position Control";
        } else if (isVelOngoing) {
            MB_STATUS = "Motor Velocity Control";
        } else if (isVelOngoing) {
            MB_STATUS = "Motor Current Control";
        } else {
            MB_STATUS = "IDLE";
        }
        /* Gripper Status */
        if (isGrpInitOngoing) {
            MB_STATUS = "Gripper Initialize";
        } else if (isObjectGrasp) {
            MB_STATUS = "Object grasped";
        } else if (isGrpOpening) {
            MB_STATUS = "Gripper Open";
        } else if (isGrpClosing) {
            MB_STATUS = "Gripper Close";
        }
    } else {
        MB_STATUS = "Motor Disabled";
    }

    if (grpDirection) {
        MB_GRP_DIR = "True";
    } else {
        MB_GRP_DIR = "False";
    }

    if (isFaultOccured) {
        MB_STATUS = "Fault occured";
    }

    static bool isMotorEnable_before    = isMotorEnable;
    static bool isGrpInitOngoing_before = isGrpInitOngoing;
    static bool isPosOngoing_before     = isPosOngoing;
    static bool isVelOngoing_before     = isVelOngoing;
    static bool isTorOngoing_before     = isTorOngoing;
    static bool isGrpOpening_before     = isGrpOpening;
    static bool isGrpClosing_before     = isGrpClosing;
    static bool isObjectGrasp_before    = isObjectGrasp;
    static bool isFaultOccured_before   = isFaultOccured;

    auto printIfChanged = [] (bool input_boolean, bool &before_boolean, std::string str_on, std::string str_off) {
        std::string retVal;

        if (input_boolean != before_boolean) {
            retVal = input_boolean ? str_on : str_off;
            std::cout << retVal << std::endl;
        }

        before_boolean = input_boolean;
        return retVal;
    };

    //MB_STATUS = printIfChanged(isMotorEnable,    isMotorEnable_before,    "[Motor][Enable]",                 "[Motor][Disable]");
    // printIfChanged(isGrpInitOngoing, isGrpInitOngoing_before, "[Gripper][Init]",                 "[Gripper][Disable]");
    // printIfChanged(isPosOngoing,     isPosOngoing_before,     "[Motor][Position OnGoing]",       "[Motor][Position Finished]");
    // printIfChanged(isVelOngoing,     isVelOngoing_before,     "[Motor][Velocity Control Start]", "[Motor][Velocity Control End]");
    // printIfChanged(isTorOngoing,     isTorOngoing_before,     "[Motor][Torque Control Start]",   "[Motor][Torque Control End]");
    // printIfChanged(isGrpOpening,     isGrpOpening_before,     "[Gripper][Opening]",              "[Gripper][Opened]");
    // printIfChanged(isGrpClosing,     isGrpClosing_before,     "[Gripper][Clsoing]",              "[Gripper][Closed]");
    printIfChanged(isObjectGrasp,    isObjectGrasp_before,    "[Gripper][Grasping]",                "[Gripper][Grasp Release]");
    //printIfChanged(isFaultOccured,   isFaultOccured_before,   "[Motor][Fault Now]",              "[Motor][Fault Occured]");

    msg_.angle   = (int16_t)mb_faultNow;
    msg_.current = (int16_t)mb_torque;
    msg_.velocity = (int16_t)mb_velocity;
    msg_.is_motor_enable = isMotorEnable;
    msg_.is_grp_init_ongoing = isGrpInitOngoing;
    msg_.is_pos_ongoing = isPosOngoing;
    msg_.is_vel_ongoing = isVelOngoing;
    msg_.is_tor_ongoing = isTorOngoing;
    msg_.is_grp_opening = isGrpOpening;
    msg_.is_grp_closing = isGrpClosing;
    msg_.grp_direction = grpDirection;
    msg_.is_object_grasp = isObjectGrasp;
    msg_.is_fault_occured = isFaultOccured;

    grp_state_publisher_->publish(msg_);

    return true;
}

void GripperNode::toggleCheckValue() {
    setReadMode(!read_mode_);
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

    driverDisable();
    modbusRelease();

    Q_EMIT rclcpp::shutdown();
}

}   // end of namespace

