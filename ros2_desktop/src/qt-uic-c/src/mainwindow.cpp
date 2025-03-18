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
    connect(ui->pushButton_pos,&QPushButton::clicked,this,&MainWindow::PushButton_clicked_pos);
    connect(ui->pushButton_att,&QPushButton::clicked,this,&MainWindow::PushButton_clicked_att);
    connect(ui->pushButton_vel_n,&QPushButton::clicked,this,&MainWindow::PushButton_clicked_vel_n);
    connect(ui->pushButton_vel_b,&QPushButton::clicked,this,&MainWindow::PushButton_clicked_vel_b);
    connect(ui->pushButton_vel,&QPushButton::clicked,this,&MainWindow::PUshButton_clicked_vel);

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

void MainWindow::PushButton_clicked_pos()
{
    m_list = &(m_joy_ctrl->m_list_pos);
    is_first = true;
    QVector<QPointF> points;
    m_x_series->replace(points);
    m_y_series->replace(points);
    m_z_series->replace(points);
    m_x_series->setName("x轴位置");
    m_y_series->setName("y轴位置");
    m_z_series->setName("z轴位置");
}

void MainWindow::PushButton_clicked_att()
{
    m_list = &(m_joy_ctrl->m_list_att);
    is_first = true;
    QVector<QPointF> points;
    m_x_series->replace(points);
    m_y_series->replace(points);
    m_z_series->replace(points);
    m_x_series->setName("x轴姿态");
    m_y_series->setName("y轴姿态");
    m_z_series->setName("z轴姿态");
}

void MainWindow::PushButton_clicked_vel_n()
{
    m_list = &(m_joy_ctrl->m_list_vel_n);
    is_first = true;
    QVector<QPointF> points;
    m_x_series->replace(points);
    m_y_series->replace(points);
    m_z_series->replace(points);
    m_x_series->setName("世界坐标x轴速度");
    m_y_series->setName("世界坐标y轴速度");
    m_z_series->setName("世界坐标z轴速度");
}

void MainWindow::PushButton_clicked_vel_b()
{
    m_list = &(m_joy_ctrl->m_list_vel_b);
    is_first = true;
    QVector<QPointF> points;
    m_x_series->replace(points);
    m_y_series->replace(points);
    m_z_series->replace(points);
    m_x_series->setName("机器坐标x轴速度");
    m_y_series->setName("机器坐标y轴速度");
    m_z_series->setName("机器坐标z轴速度");
}

void MainWindow::PUshButton_clicked_vel()
{
    m_list = &(m_joy_ctrl->m_list_ang_vel);
    is_first = true;
    QVector<QPointF> points;
    m_x_series->replace(points);
    m_y_series->replace(points);
    m_z_series->replace(points);
    m_x_series->setName("x轴角速度");
    m_y_series->setName("y轴角速度");
    m_z_series->setName("z轴角速度");
}

void MainWindow::onTimerOut()
{   
    //static int count = 0;
    static float count_max[3] = {0,0,0};
    static float count_min[3] = {0,0,0};
    static double time_start = 0;
    //m_list = &(m_joy_ctrl->m_list_att);

    if(!m_joy_ctrl->m_list_pos.empty()){
        ui->label_pos_x->setNum(m_joy_ctrl->m_list_pos[m_joy_ctrl->m_list_pos.size() - 1].getData1());
        ui->label_pos_y->setNum(m_joy_ctrl->m_list_pos[m_joy_ctrl->m_list_pos.size() - 1].getData2());
        ui->label_pos_z->setNum(m_joy_ctrl->m_list_pos[m_joy_ctrl->m_list_pos.size() - 1].getData3());  
    }
    if(!m_joy_ctrl->m_list_att.empty()){
        ui->label_att_r->setNum(m_joy_ctrl->m_list_att[m_joy_ctrl->m_list_att.size() - 1].getData1());
        ui->label_att_y->setNum(m_joy_ctrl->m_list_att[m_joy_ctrl->m_list_att.size() - 1].getData2());
        ui->label_att_p->setNum(m_joy_ctrl->m_list_att[m_joy_ctrl->m_list_att.size() - 1].getData3());
    }
    if(!m_joy_ctrl->m_list_vel_n.empty()){
        ui->label_vel_x_n->setNum(m_joy_ctrl->m_list_vel_n[m_joy_ctrl->m_list_vel_n.size() - 1].getData1());
        ui->label_vel_y_n->setNum(m_joy_ctrl->m_list_vel_n[m_joy_ctrl->m_list_vel_n.size() - 1].getData2());
        ui->label_vel_z_n->setNum(m_joy_ctrl->m_list_vel_n[m_joy_ctrl->m_list_vel_n.size() - 1].getData3());
    }
    if(!m_joy_ctrl->m_list_vel_b.empty()){
        ui->label_vel_x_b->setNum(m_joy_ctrl->m_list_vel_b[m_joy_ctrl->m_list_vel_b.size() - 1].getData1());
        ui->label_vel_y_b->setNum(m_joy_ctrl->m_list_vel_b[m_joy_ctrl->m_list_vel_b.size() - 1].getData2());
        ui->label_vel_z_b->setNum(m_joy_ctrl->m_list_vel_b[m_joy_ctrl->m_list_vel_b.size() - 1].getData3());
    }
    if(!m_joy_ctrl->m_list_ang_vel.empty()){
        ui->label_vel_r->setNum(m_joy_ctrl->m_list_ang_vel[m_joy_ctrl->m_list_ang_vel.size() - 1].getData1());
        ui->label_vel_p->setNum(m_joy_ctrl->m_list_ang_vel[m_joy_ctrl->m_list_ang_vel.size() - 1].getData2());
        ui->label_vel_y->setNum(m_joy_ctrl->m_list_ang_vel[m_joy_ctrl->m_list_ang_vel.size() - 1].getData3());
    }

    QVector<DataWithTimestamp> list;
    if(m_list != &(m_joy_ctrl->m_list_att))m_joy_ctrl->m_list_att.swap(list);
    if(m_list!= &(m_joy_ctrl->m_list_pos))m_joy_ctrl->m_list_pos.swap(list);
    if(m_list!= &(m_joy_ctrl->m_list_vel_n))m_joy_ctrl->m_list_vel_n.swap(list);
    if(m_list!= &(m_joy_ctrl->m_list_vel_b))m_joy_ctrl->m_list_vel_b.swap(list);
    if(m_list!= &(m_joy_ctrl->m_list_ang_vel))m_joy_ctrl->m_list_ang_vel.swap(list);

    if(is_first){
        is_first = false;
        time_start = m_list->at(0).getTimeInSeconds();
    }
    int listLength = m_list->length();
    // 遍历前 listLength 个元素
    for (int i = 0; i < listLength; ++i) {
        DataWithTimestamp data = (*m_list)[i];
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
    double max_time = 10;
    if((*m_list)[listLength-1].getTimeInSeconds()-time_start>10){
        max_time = (*m_list)[listLength-1].getTimeInSeconds()-time_start;
    }
    double min_time = max_time-10;

    /*while((m_x_series->at(1).x()< min_time)){
        m_x_series->remove(0);
        m_y_series->remove(0);
        m_z_series->remove(0);
    }*/

    if(m_x_series->count() > 1) {
        int remove_count = 0;
        // 预计算需要删除的点数
        while((remove_count < m_x_series->count()-1) && 
              (m_x_series->at(remove_count+1).x() < min_time)) {
            remove_count++;
        }
        // 批量删除保证数据同步
        if(remove_count > 0) {
            m_x_series->removePoints(0, remove_count);
            m_y_series->removePoints(0, remove_count);
            m_z_series->removePoints(0, remove_count);
        }
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
        m_x_series->append(QPointF((*m_list)[0].getTimeInSeconds()-time_start, (*m_list)[0].getData1()));
        m_y_series->append(QPointF((*m_list)[0].getTimeInSeconds()-time_start, (*m_list)[0].getData2()));
        m_z_series->append(QPointF((*m_list)[0].getTimeInSeconds()-time_start, (*m_list)[0].getData3()));
        m_list->erase(m_list->begin());
    }


    float max[3] = {0,0,0};
    float min[3] = {0,0,0};
    for (int i = 0; i < m_x_series->count(); ++i) {
        float value = m_x_series->at(i).y();
        if (value > max[0]) {
            max[0] = value;
        }
        if (value < min[0]) {
            min[0] = value;
        }
        value = m_y_series->at(i).y();
        if (value > max[1]) {
            max[1] = value;
        }
        if (value < min[1]) {
            min[1] = value;
        }
        value = m_z_series->at(i).y();
        if (value > max[2]) {
            max[2] = value;
        }
        if (value < min[2]) {
            min[2] = value;
        }
    }
    for (int i = 0; i < 3; ++i) {
        count_max[i] = max[i];
    }
    for (int i = 0; i < 3; ++i) {
        count_min[i] = min[i];
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
    m_x_series->setPointsVisible(false); //数据点显示
    m_y_series->setPointsVisible(false); 
    m_z_series->setPointsVisible(false);
    m_x_series->setName("x轴姿态");
    m_y_series->setName("y轴姿态");
    m_z_series->setName("z轴姿态");
    
    m_x_chart = new QChart();
    m_y_chart = new QChart();
    m_z_chart = new QChart();
    m_x_chart->addAxis(m_x_axisX, Qt::AlignBottom);
    m_x_chart->addAxis(m_x_axisY, Qt::AlignLeft);
    m_x_chart->addSeries(m_x_series);
    //m_x_chart->setAnimationOptions(QChart::SeriesAnimations);//平滑过渡
    m_y_chart->addAxis(m_y_axisX, Qt::AlignBottom);
    m_y_chart->addAxis(m_y_axisY, Qt::AlignLeft);
    m_y_chart->addSeries(m_y_series);
    //m_y_chart->setAnimationOptions(QChart::SeriesAnimations);
    m_z_chart->addAxis(m_z_axisX, Qt::AlignBottom);
    m_z_chart->addAxis(m_z_axisY, Qt::AlignLeft);
    m_z_chart->addSeries(m_z_series);
    //m_z_chart->setAnimationOptions(QChart::SeriesAnimations);
    //m_x_chart->setRenderHint(QPainter::Antialiasing, true);//抗锯齿
    m_x_chart->setMargins(QMargins(0, 0, 0, 0));//删除留白是图表紧凑
    m_y_chart->setMargins(QMargins(0, 0, 0, 0));
    m_z_chart->setMargins(QMargins(0, 0, 0, 0));

    //m_z_chart->setBackgroundRoundness(0);

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

void MainWindow::Update_time(uint64_t &time_rov, int64_t &time_diff){
    ui->time_rov->setNum(static_cast<double>(time_rov) / 1000000.0);
    ui->time_diff->setNum(static_cast<double>(time_diff) / 1000000.0);
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
