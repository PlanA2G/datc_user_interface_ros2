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

#include "gripper_test/ui_gripper_window.h"
#include "gripper_test.hpp"

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

private:
    Ui::MainWindowDesign ui;
	QTimer *timer_;
    GripperNode *grp_node_;
};

}
#endif // ur_ui_MAIN_WINDOW_H
