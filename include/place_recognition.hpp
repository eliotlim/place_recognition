#ifndef PLACE_RECOGNITION_H
#define PLACE_RECOGNITION_H

#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

const char* VERSION_STRING = "0.0.4";

void trigger_callback (const std_msgs::String &msg);
void image_callback (const sensor_msgs::Image::ConstPtr &msg);

#endif
