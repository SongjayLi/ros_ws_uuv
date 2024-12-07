#include "mainwindow.h"
#include "ros_joy.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent , joy_ctrl *joy_ctrl)
    : QMainWindow(parent)
    , m_joy_ctrl(joy_ctrl)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    connect(ui->pushButton_1,&QPushButton::clicked,this,&MainWindow::PushButton_clicked);
    //connect(m_joy,&joy_ctrl::Update_Show,this,&MainWindow::Update_num);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::PushButton_clicked()
{
    this->close();
}

void MainWindow::Update_input_num(_Float32* num)
{
    ui->label_throttle->setNum((num[1]+1)/2);
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
