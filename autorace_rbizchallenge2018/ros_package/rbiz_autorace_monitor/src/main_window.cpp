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

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/rbiz_autorace_monitor/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rbiz_autorace_monitor {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	/*********************
  ** Auto Start
  **********************/
	if (!qnode.init("http://localhost:11311/",
						"localhost"))
	{
		qnode.log(QNode::Info, "Couldn't find the ros master. ");

		close();
		// log(Info,std::string("Couldn't find the ros master. "));
		// ui.pButtonMission1->setEnabled(false);
	}

	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    // QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();


	// setWindowIcon(QIcon(":/images/icon.png"));
	// ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(fnUpdateReadyLap()), this, SLOT(fnReadyLapTime()));


		// QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(fnUpdateLapTime()));


	/*********************
	** Logging
	**********************/
	// ui.view_logging->setModel(qnode.loggingModel());
    // QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

		// while(1)
		// {
		// 	;
		// }

  // if ( ui.checkbox_remember_settings->isChecked() ) {
  	// on_button_connect_clicked(true);
  // }
	// on_pButtonMission1_clicked(true);
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

// void MainWindow::showNoMasterMessage() {
// 	QMessageBox msgBox;
// 	msgBox.setText("Couldn't find the ros master.");
// 	msgBox.exec();
//     close();
// }

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */


void MainWindow::on_pushButton_clicked(bool check)
{
    qnode.pbResetMsg();
}

void MainWindow::fnReadyLapTime()
{
    // handle ui of ready lap time
    // ..
    if (qnode.stopwatchStatus == 0)
    {
        ui.ready_min_1->display(1);
        ui.ready_min_2->display(0);
        ui.ready_sec_1->display(0);
        ui.ready_sec_2->display(0);
        ui.ready_ms_1->display(0);
        ui.ready_ms_2->display(0);

        ui.mission_min_1->display(0);
        ui.mission_min_2->display(0);
        ui.mission_sec_1->display(0);
        ui.mission_sec_2->display(0);
        ui.mission_ms_1->display(0);
        ui.mission_ms_2->display(0);


        ui.ready_min_1->setPalette(Qt::lightGray);
        ui.ready_min_2->setPalette(Qt::lightGray);
        ui.ready_sec_1->setPalette(Qt::lightGray);
        ui.ready_sec_2->setPalette(Qt::lightGray);
        ui.ready_ms_1->setPalette(Qt::lightGray);
        ui.ready_ms_2->setPalette(Qt::lightGray);

        ui.mission_min_1->setPalette(Qt::white);
        ui.mission_min_2->setPalette(Qt::white);
        ui.mission_sec_1->setPalette(Qt::white);
        ui.mission_sec_2->setPalette(Qt::white);
        ui.mission_ms_1->setPalette(Qt::white);
        ui.mission_ms_2->setPalette(Qt::white);


    }
    else if(qnode.stopwatchStatus == 1)
    {
        if ((int)(10 - ((int)(qnode.lap_time / 60000.0) % 10)) > 0)
        {
            ui.ready_min_1->display(0);
            ui.ready_min_2->display(((int)(9-((int)(qnode.lap_time / 60000.0) % 10))));
            ui.ready_sec_1->display(((int)(5-((int)(qnode.lap_time / 10000.0) % 6))));
            ui.ready_sec_2->display(((int)(9-((int)(qnode.lap_time / 1000.0) % 10))));
            ui.ready_ms_1->display(((int)(9-((int)qnode.lap_time % 1000 / 100.0))));
        }

        else
        {
            ui.ready_min_1->display(1);
            ui.ready_min_2->display(((int)((int)(60000.0 - qnode.lap_time / 60000.0) % 10)));
            ui.ready_sec_1->display(0);
            ui.ready_sec_2->display(0);
            ui.ready_ms_1->display(0);
        }
    }

    else if(qnode.stopwatchStatus == 2)
    {
        ui.mission_min_1->setPalette(Qt::green);
        ui.mission_min_2->setPalette(Qt::green);
        ui.mission_sec_1->setPalette(Qt::green);
        ui.mission_sec_2->setPalette(Qt::green);
        ui.mission_ms_1->setPalette(Qt::green);
        ui.mission_ms_2->setPalette(Qt::green);
    }

    else if(qnode.stopwatchStatus == 3)
    {

        ui.mission_min_1->display(0);
        ui.mission_min_2->display((int)(((int)(qnode.mission_time / 60000.0) % 10)));
        ui.mission_sec_1->display((int)(((int)(qnode.mission_time / 10000.0) % 6)));
        ui.mission_sec_2->display((int)(((int)(qnode.mission_time / 1000.0) % 10)));
        ui.mission_ms_1->display((int)(((int)qnode.mission_time % 1000 / 100.0)));


    }

    else if(qnode.stopwatchStatus == 4)
    {
        ui.mission_min_1->setPalette(Qt::blue);
        ui.mission_min_2->setPalette(Qt::blue);
        ui.mission_sec_1->setPalette(Qt::blue);
        ui.mission_sec_2->setPalette(Qt::blue);
        ui.mission_ms_1->setPalette(Qt::blue);
        ui.mission_ms_2->setPalette(Qt::blue);
    }

    else if(qnode.stopwatchStatus == 5)
    {
        ui.mission_min_1->setPalette(Qt::red);
        ui.mission_min_2->setPalette(Qt::red);
        ui.mission_sec_1->setPalette(Qt::red);
        ui.mission_sec_2->setPalette(Qt::red);
        ui.mission_ms_1->setPalette(Qt::red);
        ui.mission_ms_2->setPalette(Qt::red);
    }


    ui.ready_min_1->setAutoFillBackground(true);
    ui.ready_min_2->setAutoFillBackground(true);
    ui.ready_sec_1->setAutoFillBackground(true);
    ui.ready_sec_2->setAutoFillBackground(true);
    ui.ready_ms_1->setAutoFillBackground(true);
    ui.ready_ms_2->setAutoFillBackground(true);

    ui.mission_min_1->setAutoFillBackground(true);
    ui.mission_min_2->setAutoFillBackground(true);
    ui.mission_sec_1->setAutoFillBackground(true);
    ui.mission_sec_2->setAutoFillBackground(true);
    ui.mission_ms_1->setAutoFillBackground(true);
    ui.mission_ms_2->setAutoFillBackground(true);


}



// void MainWindow::on_button_connect_clicked(bool check ) {
// 	if ( ui.checkbox_use_environment->isChecked() ) {
// 		if ( !qnode.init() ) {
// 			showNoMasterMessage();
// 		} else {
// 			ui.button_connect->setEnabled(false);
// 		}
// 	} else {
// 		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
// 				   ui.line_edit_host->text().toStdString()) ) {
// 			showNoMasterMessage();
// 		} else {
// 			ui.button_connect->setEnabled(false);
// 			ui.line_edit_master->setReadOnly(true);
// 			ui.line_edit_host->setReadOnly(true);
// 			ui.line_edit_topic->setReadOnly(true);
// 		}
// 	}
// }


// void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
// 	bool enabled;
// 	if ( state == 0 ) {
// 		enabled = true;
// 	} else {
// 		enabled = false;
// 	}
// 	ui.line_edit_master->setEnabled(enabled);
// 	ui.line_edit_host->setEnabled(enabled);
// 	//ui.line_edit_topic->setEnabled(enabled);
// }

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
// void MainWindow::updateLoggingView() {
//         ui.view_logging->scrollToBottom();
// }
//
// /*****************************************************************************
// ** Implementation [Menu]
// *****************************************************************************/
//
// void MainWindow::on_actionAbout_triggered() {
//     QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
// }

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "rbiz_autorace_monitor");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    // ui.line_edit_master->setText(master_url);
    // ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    // ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    // ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	// ui.line_edit_master->setEnabled(false);
    	// ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "rbiz_autorace_monitor");
    // settings.setValue("master_url",ui.line_edit_master->text());
    // settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    // settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    // settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace rbiz_autorace_monitor
