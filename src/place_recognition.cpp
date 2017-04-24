/*
 *  Node for detecting familiar things and places
 *  Eliot Lim <eliot.lim@gmail.com>
 *  http://rosmultirgbd.wordpress.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

ros::Subscriber input_pub;
ros::Subscriber trigger_pub;
ros::Publisher output_pub;
cv_bridge::CvImagePtr image_ptr;
cv_bridge::CvImagePtr output_ptr;

void image_callback (const sensor_msgs::ImageConstPtr &msg);

int main (int argc, char** argv) {
    // ros::init must be the first call in main()
    ros::init(argc, argv, "place_recognition");
    ros::NodeHandle nh;

    std::string input_topic = nh.resolveName ("image_raw");
    std::string trigger_topic = nh.resolveName ("image_capture");
    std::string output_topic = nh.resolveName ("image_show");
    uint32_t queue_size = 1;

    nh.subscriber(input_topic, queue_size, image_callback);

    ros::spin();
}

void trigger_callback (const std_msgs::Empty &msg) {
    //TODO: Implement DB Updating and Tagging
    ROS_INFO ("DB Insertion TRIGGERED");
}

void image_callback (const sensor_msgs::ImageConstPtr &msg) {
    try {
        image_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);
        output_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);
        ROS_INFO ("cv_bridge Image Copy SUCCESS");
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR ("cv_bridge exception: %s", e.what());
    }

    output_pub.publish (output_ptr->toImageMsg());
}
