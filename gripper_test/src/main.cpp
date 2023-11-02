/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
    bool flag_init_success = false;

    QApplication app(argc, argv);
    gripper_ui::MainWindow w(argc, argv, flag_init_success);

    if (flag_init_success) {
        w.show();

        app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
        int result = app.exec();

        return result;
    } else {
        return -1;
    }
}
