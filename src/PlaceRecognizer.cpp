/*
 *  Node for detecting familiar things and places
 *  Eliot Lim <eliot.lim@gmail.com>
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

#include "PlaceRecognizer.hpp"
#include <string.h>

using namespace place_recognizer;

PlaceRecognizer::PlaceRecognizer() {
    ros::NodeHandle nh;

    // Remap and resolve Topic Names
    std::string input_topic = "place_recognition/image_raw";
    if (ros::names::remap(input_topic) != input_topic) {
      input_topic = ros::names::remap(input_topic);
    }
    std::string trigger_topic = "place_recognition/capture_trigger";
    std::string output_topic = "place_recognition/image_augmented";
    ROS_INFO ("Mapping topic image_raw to: %s", input_topic.c_str());
    ROS_INFO ("Mapping topic image_capture to: %s", trigger_topic.c_str());
    ROS_INFO ("Mapping topic image_show to: %s", output_topic.c_str());

    // Register Subscribers
    input_sub_ = nh.subscribe (input_topic, 3, &PlaceRecognizer::image_callback, this);
    trigger_sub_ = nh.subscribe (trigger_topic, 3, &PlaceRecognizer::trigger_callback, this);
    output_pub_ = nh.advertise<sensor_msgs::Image> (output_topic, 3);
    validatePubSubState();

    ROS_INFO ("Loading Vocabulary");
    loadVocabulary();
    ROS_INFO ("Place Recognizer Setup OK");
}

PlaceRecognizer::~PlaceRecognizer() {}

// Callback for Insertion Trigger
void PlaceRecognizer::trigger_callback (const std_msgs::String &msg) {
    //TODO: Implement DB Updating and Tagging
    ROS_INFO ("DB Insertion TRIGGERED");
}

// Callback for Image Messages
void PlaceRecognizer::image_callback (const sensor_msgs::ImageConstPtr &msg) {
    ROS_DEBUG ("received ImageMsg");
    try {
        // Copy the received ImageMsg to the two temporary working pointers
        image_ptr_ = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);
        output_ptr_ = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);
        ROS_DEBUG ("cv_bridge Image Copy SUCCESS");
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR ("cv_bridge exception: %s", e.what());
    }

    // TODO: Perform Database Query for matching images

    // Publish the augmented image
    output_pub_.publish (output_ptr_->toImageMsg());
}

// Load Vocabulary
void PlaceRecognizer::loadVocabulary () {
    // branching factor and depth levels - Adapted from DBoW2 Demo
    const int k = 12;
    const int L = 3;
    const DBoW2::WeightingType weight = DBoW2::TF_IDF;
    const DBoW2::ScoringType score = DBoW2::L1_NORM;

    {
        boost::shared_ptr<BriefVocabulary> temp (new BriefVocabulary (k, L, weight, score));
        vocabulary_ptr_ = temp;
    }
}

// Verify that publishers and subscribers are still alive
bool PlaceRecognizer::validatePubSubState () {
    if (!input_sub_) {
        ROS_ERROR ("PlaceRecognizer.input_sub_ is NULL");
    }
    if (!trigger_sub_) {
        ROS_ERROR ("PlaceRecognizer.trigger_sub_ is NULL");
    }
    if (!output_pub_) {
        ROS_ERROR ("PlaceRecognizer.output_pub_ is NULL");
    }
}
