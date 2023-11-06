/**
 * @file datc_ctrl.cpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief
 * @version 1.0
 * @date 2023-11-03
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "datc_ctrl.hpp"

DatcCtrl::DatcCtrl() {
}

DatcCtrl::~DatcCtrl() {
}

bool DatcCtrl::modbusInit(char *port_name, uint slave_address) {
    return mbc_.modbusInit(port_name, slave_address);
}

bool DatcCtrl::modbusRelease() {
    mbc_.modbusRelease();
    return true;
}

bool DatcCtrl::modbusSlaveChange(uint slave_addr) {
    return mbc_.slaveChange(slave_addr);
}

bool DatcCtrl::motorEnable() {
    return command(DATC_COMMAND::MOTOR_ENABLE);
}

bool DatcCtrl::motorStop() {
    return command(DATC_COMMAND::MOTOR_STOP);
}

bool DatcCtrl::motorDisable() {
    return command(DATC_COMMAND::MOTOR_DISABLE);
}

bool DatcCtrl::motorPosCtrl(int16_t pos_deg, uint16_t duration) {
    std::string error_prefix = "[Motor Position Control]";
    checkDurationRange(error_prefix, duration);
    return command(DATC_COMMAND::MOTOR_POSITION_CONTROL, pos_deg, duration);
}
bool DatcCtrl::motorVelCtrl(int16_t velocity, uint16_t duration) {
    std::string error_prefix = "[Motor Velocity Control]";
    checkDurationRange(error_prefix, duration);
    return command(DATC_COMMAND::MOTOR_VELOCITY_CONTROL, velocity, duration);
}
bool DatcCtrl::motorCurCtrl(int16_t current, uint16_t duration) {
    std::string error_prefix = "[Motor Current Control]";
    checkDurationRange(error_prefix, duration);
    return command(DATC_COMMAND::MOTOR_CURRENT_CONTROL, current, duration);
}

bool DatcCtrl::setModbusAddr(uint16_t slave_addr) {
    // TODO: modbus addr 범위 지정 필요

    return command(DATC_COMMAND::CHANGE_MODBUS_ADDRESS, slave_addr);
}

bool DatcCtrl::grpInitialize() {
    return command(DATC_COMMAND::GRIPPER_INITIALIZE);
}

bool DatcCtrl::grpOpen() {
    return command(DATC_COMMAND::GRIPPER_OPEN);
}

bool DatcCtrl::grpClose() {
    return command(DATC_COMMAND::GRIPPER_CLOSE);
}

bool DatcCtrl::setFingerPos(uint16_t finger_pos) {
    std::string error_prefix = "[Set Finger Position]";

    if (finger_pos < kFingerPosMin) {
        printf("%s Invalid range of finger position ( < %d)", error_prefix.c_str(), kFingerPosMin);
        finger_pos = kFingerPosMin;
    } else if (finger_pos > kFingerPosMax) {
        printf("%s Invalid range of finger position ( > %d)", error_prefix.c_str(), kFingerPosMax);
        finger_pos = kFingerPosMax;
    }

    return command(DATC_COMMAND::SET_FINGER_POSITION, finger_pos);
}

bool DatcCtrl::vacuumGrpOn() {
    return command(DATC_COMMAND::VACUUM_GRIPPER_ON);
}

bool DatcCtrl::vacuumGrpOff() {
    return command(DATC_COMMAND::VACUUM_GRIPPER_OFF);
}

bool DatcCtrl::setMotorTorque(uint16_t torque_ratio) {
    std::string error_prefix = "[Set Motor Torque]";

    if (torque_ratio < kTorqueRatioMin) {
        printf("%s Motor torque is too low ( < %d)", error_prefix.c_str(), kTorqueRatioMin);
        torque_ratio = kTorqueRatioMin;
    } else if (torque_ratio > kTorqueRatioMax) {
        printf("%s Motor torque is too high ( > %d)", error_prefix.c_str(), kTorqueRatioMax);
        torque_ratio = kTorqueRatioMax;
    }

    return command(DATC_COMMAND::SET_MOTOR_TORQUE, torque_ratio);
}

bool DatcCtrl::setMotorSpeed (uint16_t speed_ratio) {
    std::string error_prefix = "[Set Motor Torque]";

    if (speed_ratio < kSpeedRatioMin) {
        printf("%s Motor torque is too low ( < %d)", error_prefix.c_str(), kSpeedRatioMin);
        speed_ratio = kSpeedRatioMin;
    } else if (speed_ratio > kSpeedRatioMax) {
        printf("%s Motor torque is too high ( > %d)", error_prefix.c_str(), kSpeedRatioMax);
        speed_ratio = kSpeedRatioMax;
    }

    return command(DATC_COMMAND::SET_MOTOR_SPEED, speed_ratio);
}

bool DatcCtrl::readDatcData() {
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

    // Read input register //
    uint16_t reg_addr = 10;
    uint16_t reg_num  = 8;
    std::vector<uint16_t> reg;

    mbc_.recvData(reg_addr, reg_num, reg);

    uint16_t status    = reg[0];
    status_.motor_pos  = reg[1];
    status_.motor_cur  = reg[2];
    status_.motor_vel  = reg[3];
    status_.finger_pos = reg[4];
    status_.voltage    = reg[7];

    status_.status_str = "Motor Disabled";

    for (int i = 0; i < 16; i++) {
        if (status_info.find(i) != status_info.end()) {
            if (status & (0x01 << i)) {
                status_info[i][0] = "true";
                status_.status_str = status_info[i][1];
            }
        }
    }

    return true;
}

bool DatcCtrl::checkDurationRange(std::string error_prefix, uint16_t &duration) {
    if (duration < kDurationMin) {
        printf("%s Duration is too short ( < %dms)", error_prefix.c_str(), kDurationMin);
        duration = kDurationMin;
        return false;
    } else if (duration > kDurationMax) {
        printf("%s Duration is too long ( > %dms)", error_prefix.c_str(), kDurationMax);
        duration = kDurationMax;
        return false;
    }

    return true;
}

bool DatcCtrl::command(DATC_COMMAND cmd, uint16_t value_1, uint16_t value_2) {
    switch (cmd) {
        case DATC_COMMAND::MOTOR_ENABLE:
            return SEND_CMD(cmd);

        case DATC_COMMAND::MOTOR_STOP:
            return SEND_CMD(cmd);

        case DATC_COMMAND::MOTOR_DISABLE:
            return SEND_CMD(cmd);

        case DATC_COMMAND::MOTOR_POSITION_CONTROL:
            return SEND_CMD_VECTOR(std::vector<uint16_t> ({(uint16_t) cmd, value_1, value_2}));

        case DATC_COMMAND::MOTOR_VELOCITY_CONTROL:
            return SEND_CMD_VECTOR(std::vector<uint16_t> ({(uint16_t) cmd, value_1, value_2}));

        case DATC_COMMAND::MOTOR_CURRENT_CONTROL:
            return SEND_CMD_VECTOR(std::vector<uint16_t> ({(uint16_t) cmd, value_1, value_2}));

        case DATC_COMMAND::CHANGE_MODBUS_ADDRESS:
            return SEND_CMD_VECTOR(std::vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::GRIPPER_INITIALIZE:
            return SEND_CMD(cmd);

        case DATC_COMMAND::GRIPPER_OPEN:
            return SEND_CMD(cmd);

        case DATC_COMMAND::GRIPPER_CLOSE:
            return SEND_CMD(cmd);

        case DATC_COMMAND::SET_FINGER_POSITION:
            return SEND_CMD_VECTOR(std::vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::VACUUM_GRIPPER_ON:
            return SEND_CMD(cmd);

        case DATC_COMMAND::VACUUM_GRIPPER_OFF:
            return SEND_CMD(cmd);

        case DATC_COMMAND::SET_MOTOR_TORQUE:
            return SEND_CMD_VECTOR(std::vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::SET_MOTOR_SPEED:
            return SEND_CMD_VECTOR(std::vector<uint16_t> ({(uint16_t) cmd, value_1}));

        default:
            COUT("Error: Undefined command.");
            return false;
    }
}
