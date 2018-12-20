
#ifndef _MAINWINDOW_H
#define _MAINWINDOW_H

#include "ui_MainWindow.h"
#include <ros/ros.h>

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow();
    virtual ~MainWindow();
    
public slots:
    void OnLeftClicked();
    void OnRightClicked();
    void OnUpClicked();
    void OnDownClicked();
        
private:
    Ui::MainWindow widget;
    ros::NodeHandlePtr nh;
    ros::Publisher pub_cmd_vel;
    double linear,
           angular,
           linear_scale,
           angular_scale;
    
    void TranslateAndPublish();
};

#endif /* _MAINWINDOW_H */
