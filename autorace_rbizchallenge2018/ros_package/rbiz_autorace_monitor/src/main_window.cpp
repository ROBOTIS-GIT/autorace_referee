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
    QObject::connect(&qnode, SIGNAL(resetTrainingTime()), this, SLOT(readyAllTime()));
    QObject::connect(&qnode, SIGNAL(startTrainingTime()), this, SLOT(startReadyTime()));
    QObject::connect(&qnode, SIGNAL(setTrainingTime(int, int, int)), this, SLOT(setTrainingTime(int, int, int)));
    QObject::connect(&qnode, SIGNAL(readyMissionTime()), this, SLOT(readyMissionTime()));
    QObject::connect(&qnode, SIGNAL(startMissionTime(int, int, int)), this, SLOT(startMissionTime(int, int, int)));
    QObject::connect(&qnode, SIGNAL(finishMission()), this, SLOT(finishMission()));
    QObject::connect(&qnode, SIGNAL(timeOut()), this, SLOT(timeOut()));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

// void MainWindow::showNoMasterMessage() {finishMission
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

void MainWindow::readyAllTime()
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

void MainWindow::startReadyTime()
{
    // set start time
    start_ready_time_ = ros::Time::now();
}

void MainWindow::setTrainingTime(int min, int sec, int m_sec)
{
    // display training time
    ui.ready_min_1->display(min/10);
    ui.ready_min_2->display(min%10);
    ui.ready_sec_1->display(sec/10);
    ui.ready_sec_2->display(sec%10);
    ui.ready_ms_1->display(m_sec/10);
    ui.ready_ms_2->display(m_sec%10);
}

void MainWindow::resetTrainingTime()
{
    start_ready_time_ = ros::Time::now();
}

void MainWindow::readyMissionTime()
{
    ui.mission_min_1->setPalette(Qt::green);
    ui.mission_min_2->setPalette(Qt::green);
    ui.mission_sec_1->setPalette(Qt::green);
    ui.mission_sec_2->setPalette(Qt::green);
    ui.mission_ms_1->setPalette(Qt::green);
    ui.mission_ms_2->setPalette(Qt::green);
}

void MainWindow::startMissionTime(int min, int sec, int m_sec)
{
    ui.mission_min_1->display(min/10);
    ui.mission_min_2->display(min%10);
    ui.mission_sec_1->display(sec/10);
    ui.mission_sec_2->display(sec%10);
    ui.mission_ms_1->display(m_sec/10);
    ui.mission_ms_2->display(m_sec%10);
}

void MainWindow::finishMission()
{
    ui.mission_min_1->setPalette(Qt::blue);
    ui.mission_min_2->setPalette(Qt::blue);
    ui.mission_sec_1->setPalette(Qt::blue);
    ui.mission_sec_2->setPalette(Qt::blue);
    ui.mission_ms_1->setPalette(Qt::blue);
    ui.mission_ms_2->setPalette(Qt::blue);
}

void MainWindow::timeOut()
{
    ui.mission_min_1->setPalette(Qt::red);
    ui.mission_min_2->setPalette(Qt::red);
    ui.mission_sec_1->setPalette(Qt::red);
    ui.mission_sec_2->setPalette(Qt::red);
    ui.mission_ms_1->setPalette(Qt::red);
    ui.mission_ms_2->setPalette(Qt::red);
}

}  // namespace rbiz_autorace_monitor
