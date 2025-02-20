#include "mainwindow.h"
#include "ros_joy.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent , joy_ctrl *joy_ctrl)
    : QMainWindow(parent)
    , m_joy_ctrl(joy_ctrl)
    , ui(new Ui::mainWindow)
{
    ui->setupUi(this);
    _init_ui();
    
    connect(ui->pushButton_start,&QPushButton::clicked,this,&MainWindow::PushButton_clicked_start);
    connect(ui->pushButton_stop,&QPushButton::clicked,this,&MainWindow::PushButton_clicked_stop);
    //connect(m_joy,&joy_ctrl::Update_Show,this,&MainWindow::Update_num);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::PushButton_clicked_start()
{
    m_joy_ctrl->start_offboard_control();
    //this->close();
}

void MainWindow::PushButton_clicked_stop()
{
    m_joy_ctrl->stop_offboard_control();
    //this->close();
}

void MainWindow::onTimerOut()
{
    //static int count = 0;
    static float count_max[3] = {0,0,0};
    static float count_min[3] = {0,0,0};
    QVector<DataWithTimestamp>* list = &(m_joy_ctrl->m_list_att);
    int listLength = list->length();
    // 遍历前 listLength 个元素
    for (int i = 0; i < listLength; ++i) {
        DataWithTimestamp data = (*list)[i];
        // 假设 DataWithTimestamp 有三个参数 x, y, z
        float values[3] = {data.getData1(), data.getData2(), data.getData3()};
        // 更新最大最小值
        for (int j = 0; j < 3; ++j) {
            if (values[j] > count_max[j]) {
                count_max[j] = values[j];
            }
            if (values[j] < count_min[j]) {
                count_min[j] = values[j];
            }
        }
    }

    double max_time = (*list)[listLength-1].getTimeInSeconds();
    double min_time = max_time-10;
    if(max_time>10 && m_x_series->count()>0){
        while(m_x_series->at(0).x()< max_time-10){
            m_x_series->remove(0);
        }
        min_time = m_x_series->at(0).x();
    }
    m_x_axisX->setMin(min_time);
    m_x_axisX->setMax(max_time);
    m_x_axisY->setMin(count_min[0]-1);
    m_x_axisY->setMax(count_max[0]+1);
    m_y_axisX->setMin(min_time);
    m_y_axisX->setMax(max_time);
    m_y_axisY->setMin(count_min[1]-1);
    m_y_axisY->setMax(count_max[1]+1);
    m_z_axisX->setMin(min_time);
    m_z_axisX->setMax(max_time);
    m_z_axisY->setMin(count_min[2]-1);
    m_z_axisY->setMax(count_max[2]+1);

    for (int i = 0; i < listLength; ++i) {
        m_x_series->append(QPointF((*list)[0].getTimeInSeconds(), (*list)[0].getData1()));
        m_y_series->append(QPointF((*list)[0].getTimeInSeconds(), (*list)[0].getData2()));
        m_z_series->append(QPointF((*list)[0].getTimeInSeconds(), (*list)[0].getData3()));
        list->erase(list->begin());
    }

    for (int i = 0; i < m_x_series->count(); ++i) {
        float value = m_x_series->at(i).y();
        if (value > count_max[0]) {
            count_max[0] = value;
        }
        if (value < count_min[0]) {
            count_min[0] = value;
        }
        value = m_y_series->at(i).y();
        if (value > count_max[1]) {
            count_max[1] = value;
        }
        if (value < count_min[1]) {
            count_min[1] = value;
        }
        value = m_z_series->at(i).y();
        if (value > count_max[2]) {
            count_max[2] = value;
        }
        if (value < count_min[2]) {
            count_min[2] = value;
        }
    }
  
}

void MainWindow::_init_ui()
{
    ui->label_picture_rov->setStyleSheet(QString::fromUtf8("border-image: url(other/bluerov.png);"));

    m_x_axisX = new QValueAxis();
    m_x_axisY = new QValueAxis();
    m_y_axisX = new QValueAxis();
    m_y_axisY = new QValueAxis();
    m_z_axisX = new QValueAxis();
    m_z_axisY = new QValueAxis();
    m_x_axisX->setTitleText("time");
    m_x_axisY->setTitleText("x");
    m_x_axisX->setRange(0, X_AXIS_MAX_X);
    m_x_axisY->setRange(0, X_AXIS_MAX_Y);
    m_y_axisX->setTitleText("time");
    m_y_axisY->setTitleText("y");
    m_y_axisX->setRange(0, Y_AXIS_MAX_X);
    m_y_axisY->setRange(0, Y_AXIS_MAX_Y);
    m_z_axisX->setTitleText("time");
    m_z_axisY->setTitleText("z");
    m_z_axisX->setRange(0, Z_AXIS_MAX_X);
    m_z_axisY->setRange(0, Z_AXIS_MAX_Y);

    m_x_series = new QLineSeries();
    m_y_series = new QLineSeries();
    m_z_series = new QLineSeries();
    m_x_series->setPointsVisible(true); //数据点显示
    m_y_series->setPointsVisible(true); 
    m_z_series->setPointsVisible(true);

    m_x_chart = new QChart();
    m_y_chart = new QChart();
    m_z_chart = new QChart();
    m_x_chart->addAxis(m_x_axisX, Qt::AlignBottom);
    m_x_chart->addAxis(m_x_axisY, Qt::AlignLeft);
    m_x_chart->addSeries(m_x_series);
    m_x_chart->setAnimationOptions(QChart::SeriesAnimations);//平滑过渡
    m_y_chart->addAxis(m_y_axisX, Qt::AlignBottom);
    m_y_chart->addAxis(m_y_axisY, Qt::AlignLeft);
    m_y_chart->addSeries(m_y_series);
    m_y_chart->setAnimationOptions(QChart::SeriesAnimations);
    m_z_chart->addAxis(m_z_axisX, Qt::AlignBottom);
    m_z_chart->addAxis(m_z_axisY, Qt::AlignLeft);
    m_z_chart->addSeries(m_z_series);
    m_z_chart->setAnimationOptions(QChart::SeriesAnimations);

    m_x_series->attachAxis(m_x_axisX);
    m_x_series->attachAxis(m_x_axisY);
    m_y_series->attachAxis(m_y_axisX);
    m_y_series->attachAxis(m_y_axisY);
    m_z_series->attachAxis(m_z_axisX);
    m_z_series->attachAxis(m_z_axisY);
    
    ui->view_x->setChart(m_x_chart);
    ui->view_y->setChart(m_y_chart);
    ui->view_z->setChart(m_z_chart);

    m_timer = new QTimer(this);
    m_timer->setInterval(200);
    connect(m_timer, &QTimer::timeout, this, &MainWindow::onTimerOut);
    m_timer->start();
}

void MainWindow::Update_input_num(_Float32* num)
{
    ui->label_throttle->setNum(num[1]);
    ui->label_yaw->setNum(-num[0]);
    ui->label_pitch->setNum(num[4]);
    ui->label_roll->setNum(-num[3]);
}

void MainWindow::Update_actuator(std::array<float, 12UL> num)
{
    ui->label_motor1->setNum(num[0]);
    ui->label_motor2->setNum(num[1]);
    ui->label_motor3->setNum(num[2]);
    ui->label_motor4->setNum(num[3]);
    ui->label_motor5->setNum(num[4]);
    ui->label_motor6->setNum(num[5]);
}

void MainWindow::init_first()
{
    ui->label_motor1->setNum(0);
    ui->label_motor2->setNum(0);
    ui->label_motor3->setNum(0);
    ui->label_motor4->setNum(0);
    ui->label_motor5->setNum(0);
    ui->label_motor6->setNum(0);
    ui->label_pitch->setNum(0);
    ui->label_roll->setNum(0);
    ui->label_throttle->setNum(0);
    ui->label_yaw->setNum(0);
}
