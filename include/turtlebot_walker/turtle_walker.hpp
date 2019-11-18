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
 *  @file    turtle_walker.hpp
 *  @author  Kartik Madhira
 *  @copyright MIT License
 *
 *  @brief  ENPM808X :Assignment to implement TurtleBot walker algorithm
 *  This file contains TurtleWalker class method declarations. This implements
 *  a roomba like algorithm for turtlebot using callback to laserscan.
 */


#pragma once

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

/**
 *  @brief Class TurtleWalker
 *
 *  This class subscribes to laserscan msgs to get 2D depths
 *  and check for collision
 */
class TurtleWalker {
 private:
    ros::NodeHandle handler;
    ros::Subscriber sub;
    ros::Publisher pub;
    bool collisionCheck;
    geometry_msgs::Twist msgTwist;
 public:
    /**
    *   @brief  Constructor for TurtleWalker class
    *   @param  none
    *   @return void
    */
    TurtleWalker();
    /**
    *   @brief  Destructor for TWalker class
    *   @param  none
    *   @return void
    */
    ~TurtleWalker();
    /**
    *   @brief  Callback for subscribing to laserScan data       
    *   @param  LaserScan pointer message
    *   @return void
    */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    /**
    *   @brief  Function for detection of obstacles
    *   @param  none
    *   @return boolean if obstacle is present or not
    */
    bool checkObstacle();
    /**
    *   @brief Function that publishes data to move turtlebot
    *   @return void
    */
    void moveBot();

};


