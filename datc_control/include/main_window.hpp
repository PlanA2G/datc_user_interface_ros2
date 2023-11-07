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

#include <QtGui>
#include <QTimer>
#include <QLineEdit>
#include <QList>
#include <QMessageBox>
#include <QMainWindow>

#include <iostream>
#include <math.h>

#include "datc_comm_interface.hpp"

#ifdef RCLCPP__RCLCPP_HPP_
#include "gripper_test/ui_gripper_window.h"
#else
#include "./ui_gripper_window.h"
#endif

using namespace std;

namespace gripper_ui {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, bool &success, QWidget *parent = 0);
	~MainWindow();

public Q_SLOTS:

    void timerCallback();

    // Enable & disable
    void pushButton_cmdEnableCallback();
    void pushButton_cmdDisableCallback();

    // Control
    void pushButton_grpPosCtrlCallback();

    void pushButton_grpInitCallback();
    void pushButton_grpOpenCallback();
    void pushButton_grpCloseCallback();
    void pushButton_vacuumGrpOnCallback();
    void pushButton_vacuumGrpOffCallback();

    void pushButton_setTorqueCallback();
    void pushButton_setSpeedCallback();

private:
#ifdef RCLCPP__RCLCPP_HPP_
    Ui::MainWindowDesign ui;
#else
    Ui::MainWindow ui;
#endif

	QTimer *timer_;
    DatcCommInterface *datc_interface_;
};

}
#endif // ur_ui_MAIN_WINDOW_H
