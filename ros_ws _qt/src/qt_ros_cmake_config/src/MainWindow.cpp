
#include "MainWindow.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

MainWindow::MainWindow():
    linear_scale(2),
    angular_scale(2),
    linear(0),
    angular(0),
    nh(new ros::NodeHandle)
{
    widget.setupUi(this);

    pub_cmd_vel = nh->advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

    connect(widget.btn_up, &QPushButton::clicked,
            this, &MainWindow::OnUpClicked);
    connect(widget.btn_left, &QPushButton::clicked,
            this, &MainWindow::OnLeftClicked);
    connect(widget.btn_right, &QPushButton::clicked,
            this, &MainWindow::OnRightClicked);
    connect(widget.btn_down, &QPushButton::clicked,
            this, &MainWindow::OnDownClicked);   
}

MainWindow::~MainWindow()
{
}

void MainWindow::TranslateAndPublish()
{
    geometry_msgs::Twist twist;
    twist.angular.z = angular_scale * angular;
    twist.linear.x = linear_scale * linear;
    pub_cmd_vel.publish(twist);
    
    linear = angular = 0;
}

void MainWindow::OnLeftClicked()
{
    angular = 1;
    this->TranslateAndPublish();
}

void MainWindow::OnRightClicked()
{
    angular = -1;
    this->TranslateAndPublish();
}

void MainWindow::OnUpClicked()
{
    linear = 1;
    this->TranslateAndPublish();
}

void MainWindow::OnDownClicked()
{
    linear = -1;
    this->TranslateAndPublish();
}