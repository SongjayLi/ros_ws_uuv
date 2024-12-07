#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
//#include "ros_joy.h"




QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class joy_ctrl;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr,joy_ctrl *joy_ctrl = nullptr);
    ~MainWindow();
    void Update_input_num(_Float32* num);
    void Update_actuator(std::array<float, 12UL> num);
    void init_first();

public slots:
    void PushButton_clicked();
    


private:
    //joy_ctrl *m_joy;
    joy_ctrl *m_joy_ctrl;
    Ui::MainWindow *ui;
    
};
#endif // MAINWINDOW_H
