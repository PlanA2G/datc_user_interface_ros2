/**
 * @file main_window.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief
 * @version 1.0
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QTimer>
#include <QLineEdit>
#include <QList>
#include <QMainWindow>

#include <iostream>
#include <math.h>

#include "datc_comm_interface.hpp"
#include "ui_main_window.h"
#include "custom_widget.hpp"

using namespace std;

namespace gripper_ui {

enum class WidgetSeq {
    MODBUS_WIDGET    = 0,
    DATC_CTRL_WIDGET = 1,
    TCP_WIDGET       = 2
};

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, bool &success, QWidget *parent = 0);
    ~MainWindow();

public Q_SLOTS:
    void timerCallback();

    // Enable & disable
    void datcEnable();
    void datcDisable();

    // Datc control
    void datcFingerPosCtrl();

    void datcInit();
    void datcOpen();
    void datcClose();
    void datcStop();
    void datcVacuumGrpOn();
    void datcVacuumGrpOff();

    void datcSetTorque();
    void datcSetSpeed();

    // Modbus RTU related
    void initModbus();
    void releaseModbus();
    void changeSlaveAddress();
    void setSlaveAddr();

#ifndef RCLCPP__RCLCPP_HPP_
    // TCP comm. related functions
    void startTcpComm();
    void stopTcpComm();
#endif

    // Auto mapping function
    void on_pushButton_select_modbus_clicked();
    void on_pushButton_select_datc_ctrl_clicked();
    void on_pushButton_select_tcp_clicked();

private:
    Ui::MainWindow *ui_;

    ModbusWidget   *modbus_widget_;
    DatcCtrlWidget *datc_ctrl_widget_;
    TcpWidget      *tcp_widget_;

    QString menu_btn_active_str_, menu_btn_inactive_str_;
    QString btn_active_str_, btn_inactive_str_;

    QTimer *timer_;
    DatcCommInterface *datc_interface_;
};

}
#endif // ur_ui_MAIN_WINDOW_H
