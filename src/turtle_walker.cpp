/**
 *  MIT License
 *
 *  Copyright (c) 2019 Kartik Madhira
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 *
 *  @file    turtle_walker.cpp
 *  @author  Kartik Madhira
 *  @copyright MIT License
 *
 *  @brief  ENPM808X :Assignment to implement TurtleBot walker algorithm
 *
 */


#include <iostream>
#include "../include/turtlebot_walker/turtle_walker.hpp" 

TurtleWalker::TurtleWalker(ros::NodeHandle &n) {
    collisionCheck = false;
    ROS_INFO_STREAM("Walker algorithm for Turtlebot initialized");
    // Publish odometry twist messages
    pub = n.advertise <geometry_msgs::Twist> ("/cmd_vel_mux/input/navi", 1000);
    // Subscribe to the laserscans from 2D lidar on top of turtlebot
    sub = n.subscribe("scan", 500, &TurtleWalker::laserCallback, this);
    //define the initial velocities
    msgTwist.linear.x = 0.0;
    msgTwist.linear.y = 0.0;
    msgTwist.linear.z = 0.0;
    msgTwist.angular.x = 0.0;
    msgTwist.angular.y = 0.0;
    msgTwist.angular.z = 0.0;
    pub.publish(msgTwist);
}

TurtleWalker::~TurtleWalker() {
   // Completely stop turtlebot
    msgTwist.linear.x = 0.0;
    msgTwist.linear.y = 0.0;
    msgTwist.linear.z = 0.0;
    msgTwist.angular.x = 0.0;
    msgTwist.angular.y = 0.0;
    msgTwist.angular.z = 0.0;
    pub.publish(msgTwist);
}


void TurtleWalker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msgTwist) {
    // Loop though the laserscan vector messages pointer
    for (int i = 0; i < msgTwist->ranges.size(); ++i) {
    // Threshold at 0.80cm to avoid collision
        if (msgTwist->ranges[i] < 0.80) {
            collisionCheck = true;
            return;
        }
    }
    collisionCheck = false;
    return;
}

bool TurtleWalker::checkObstacle() {
    return collisionCheck;
}

void TurtleWalker::moveBot() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if(checkObstacle()) {
            ROS_WARN_STREAM("Obstacle in line!, Moving.");
            // If obstacle is detected turn around
            msgTwist.linear.x = 0.0;
            msgTwist.angular.z = -0.8;
        } else {
            msgTwist.angular.z = 0.0;
            msgTwist.linear.x = 0.1;
        }
        pub.publish(msgTwist);
        ros::spinOnce();
        loop_rate.sleep();
    }
}