#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCharts/QtCharts>
#include <QTimer>
#include "ros_joy.h"



QT_BEGIN_NAMESPACE
namespace Ui {
class mainWindow;
}
QT_END_NAMESPACE

class joy_ctrl;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    QValueAxis* m_x_axisX;
    QValueAxis* m_x_axisY;
    QValueAxis* m_y_axisX;
    QValueAxis* m_y_axisY;
    QValueAxis* m_z_axisX;
    QValueAxis* m_z_axisY;
    QLineSeries* m_x_series;
    QLineSeries* m_y_series;
    QLineSeries* m_z_series;
    QChart* m_x_chart;
    QChart* m_y_chart;
    QChart* m_z_chart;

    MainWindow(QWidget *parent = nullptr,joy_ctrl *joy_ctrl = nullptr);
    ~MainWindow();
    void Update_input_num(_Float32* num);
    void Update_time(uint64_t &time_rov,int64_t &time_diff);
    void Update_actuator(std::array<float, 12UL> num);
    void init_first();

public slots:
    void PushButton_clicked_start();
    void PushButton_clicked_stop();
    void PushButton_clicked_pos();
    void PushButton_clicked_att();
    void PushButton_clicked_vel_n();
    void PushButton_clicked_vel_b();
    void PUshButton_clicked_vel();
    void onTimerOut();


private:
    //joy_ctrl *m_joy;
    joy_ctrl *m_joy_ctrl;
    QVector<DataWithTimestamp>* m_list{&(m_joy_ctrl->m_list_att)};
    bool is_first = true;
    Ui::mainWindow *ui;
    QTimer *m_timer;
    void _init_ui();
    float X_AXIS_MAX_X = 10;
    float X_AXIS_MAX_Y = 10;
    float Y_AXIS_MAX_X = 10;
    float Y_AXIS_MAX_Y = 10;
    float Z_AXIS_MAX_X = 10;
    float Z_AXIS_MAX_Y = 10;
};
#endif // MAINWINDOW_H
