/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

// #include <QtGui>
#include "main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

namespace gripper_ui {

MainWindow::MainWindow(int argc, char **argv, bool &success, QWidget *parent): QMainWindow(parent) {
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    rclcpp::init(argc, argv);

    static shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("grp");

    //setWindowIcon(QIcon(":/images/icon.png"));
    grp_node_ = new GripperNode(argc, argv, node);

    QObject::connect(grp_node_, SIGNAL(rosShutdown()), this, SLOT(close()));

    // Button mapping
    QObject::connect(ui.pushButton_cmd_enable  , SIGNAL(clicked()), this, SLOT(pushButton_cmdEnableCallback()));
    QObject::connect(ui.pushButton_cmd_disable , SIGNAL(clicked()), this, SLOT(pushButton_cmdDisableCallback()));
    QObject::connect(ui.pushButton_set_position, SIGNAL(clicked()), this, SLOT(pushButton_grpPosCtrlCallback()));

    QObject::connect(ui.pushButton_cmd_initial       , SIGNAL(clicked()), this, SLOT(pushButton_grpInitCallback()));
    QObject::connect(ui.pushButton_cmd_grp_open      , SIGNAL(clicked()), this, SLOT(pushButton_grpOpenCallback()));
    QObject::connect(ui.pushButton_cmd_grp_close     , SIGNAL(clicked()), this, SLOT(pushButton_grpCloseCallback()));
    QObject::connect(ui.pushButton_cmd_grp_vacuum_on , SIGNAL(clicked()), this, SLOT(pushButton_vacuumGrpOnCallback()));
    QObject::connect(ui.pushButton_cmd_grp_vacuum_off, SIGNAL(clicked()), this, SLOT(pushButton_vacuumGrpOffCallback()));

    if (argc >= 3) {
        string address_str = argv[2];
        uint slave_address = stoi(address_str);

        cout << "--------------------------------------" << endl;
        cout << "[INFO] Port: " << argv[1] << endl;
        cout << "[INFO] Slave address #" << slave_address << endl;
        cout << "--------------------------------------" << endl;

        if (grp_node_->init(argv[1], slave_address)) {
            timer_ = new QTimer(this);
            connect(timer_, SIGNAL(timeout()), this, SLOT(timerCallback()));
            timer_->start(200); // msec
            success = true;
        } else {
            cout << "[ERROR] Port name or slave address invlaid !" << endl;
        }
    } else {
        cout << "--------------------------------------" << endl;
        cout << "[ERROR] Port name & Slave address required !" << endl;
        cout << "--------------------------------------" << endl;
    }
}

MainWindow::~MainWindow() {
    if(grp_node_ != NULL) {
        delete grp_node_;
    }

    if(timer_ != NULL) {
        delete timer_;
    }
}

void MainWindow::timerCallback() {
    // Display
    ui.lineEdit_monitor_position-> setText(QString::number((double)grp_node_->msg_.angle / 100 , 'f', 1) + " %");
    ui.lineEdit_monitor_current -> setText(QString::number(grp_node_->msg_.current, 'd', 0) + " mA");
    ui.lineEdit_monitor_mode    -> setText(QString::fromStdString(grp_node_->MB_STATUS));
}

void MainWindow::pushButton_cmdEnableCallback() {
    grp_node_->driverEnable();
}

void MainWindow::pushButton_cmdDisableCallback() {
    grp_node_->driverDisable();
}

void MainWindow::pushButton_grpPosCtrlCallback() {
    uint16_t parameter;
    parameter = ui.doubleSpinBox_grpPos_ctrl_range_1->value() * 100;
    grp_node_->grpPosCtrl(parameter);
}

void MainWindow::pushButton_grpInitCallback() {
    grp_node_->grpInit();
}

void MainWindow::pushButton_grpOpenCallback() {
    grp_node_->grpOpen();
}

void MainWindow::pushButton_grpCloseCallback() {
    grp_node_->grpClose();
}

void MainWindow::pushButton_vacuumGrpOnCallback() {
    grp_node_->vacuumGrpOn();
}

void MainWindow::pushButton_vacuumGrpOffCallback() {
    grp_node_->vacuumGrpOff();
}

}   // end of namespace