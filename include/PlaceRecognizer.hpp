#ifndef PLACE_RECOGNIZER_H
#define PLACE_RECOGNIZER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

namespace place_recognizer {

class PlaceRecognizer {
public:

private:
    ros::Subscriber input_sub_;
    ros::Subscriber trigger_sub_;
    ros::Publisher output_pub_;
    cv_bridge::CvImagePtr image_ptr_;
    cv_bridge::CvImagePtr output_ptr_;

public:
    PlaceRecognizer ();
    ~PlaceRecognizer ();
    void trigger_callback (const std_msgs::String &msg);
    void image_callback (const sensor_msgs::Image::ConstPtr &msg);


};

}

#endif
