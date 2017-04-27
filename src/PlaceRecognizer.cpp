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

    loadDatabase();
    // TODO: Save Database if modified
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

// Load Vocabulary : Returns true if success, false if fail
bool PlaceRecognizer::loadVocabulary () {
    // branching factor and depth levels - Adapted from DBoW2 Demo
    const int k = 12;
    const int L = 3;
    const DBoW2::WeightingType weight = DBoW2::TF_IDF;
    const DBoW2::ScoringType score = DBoW2::L1_NORM;

    const std::string voc_filename = "ORBvoc.txt";

    {
        boost::shared_ptr<ORBVocabulary> temp (new ORBVocabulary (k, L, weight, score));
        vocabulary_ptr_ = temp;
    }

    try {
        /** This REQUIRES the included version of TemplatedVocabulary.h which is adapted
          * from: https://github.com/raulmur/ORB_SLAM2/raw/master/Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h
        **/
        ROS_INFO ("Loading Vocabulary");
        vocabulary_ptr_->loadFromTextFile (voc_filename);
    } catch (std::string &exception_msg) {
        ROS_WARN ("Problem loading vocabulary from file %s : %s", voc_filename.c_str(), exception_msg.c_str());
        return false;
    }

    return true;
}

// Load Database: Returns true if success, false if fail
bool PlaceRecognizer::loadDatabase () {
    std::string db_filename ("ORBdb.yml");
    bool fail = true;

    // Attempt to load the feature database from disk first.
    try {
        ROS_DEBUG ("Database - Loading - file: %s", db_filename.c_str());
        boost::shared_ptr<ORBDatabase> temp (new ORBDatabase (db_filename));
        database_ptr_ = temp;
        fail = false;
        ROS_DEBUG ("Database - Load SUCCESS");
    } catch (std::string &exception_msg) {
        ROS_WARN ("Database - Load FAIL - file: %s : %s", db_filename.c_str(), exception_msg.c_str());
    } catch (cv::Exception &exception) {
        ROS_WARN ("Database - Load FAIL - file: %s : %s", db_filename.c_str(), exception.what());
    }

    // If feature database on disk is invalid, load vocabulary and create database.
    if (fail) {
        loadVocabulary();
        boost::shared_ptr<ORBDatabase> temp (new ORBDatabase (*vocabulary_ptr_, false, 0));
        database_ptr_ = temp;
        fail = false;
        ROS_DEBUG ("Database - Create SUCCESS");
    }

    return !fail;
}

bool PlaceRecognizer::saveDatabase () {
    const std::string db_filename ("ORBdb.yml");

    // Attempt to save database to disk
    ROS_DEBUG ("Database - Saving");
    if (database_ptr_ != NULL)
        try {
            //ROS_INFO ("Database - Save - file: %s", db_filename.c_str());
            database_ptr_->save (db_filename);
            ROS_DEBUG ("Database - Save SUCCESS");
        } catch (std::string &exception_msg) {
            ROS_WARN ("Database - Save FAIL - file: %s : %s", db_filename.c_str(), exception_msg.c_str());
            return false;
        } catch (cv::Exception &exception) {
            ROS_WARN ("Database - Save FAIL - file: %s : %s", db_filename.c_str(), exception.what());
            return false;
        }
    else
        ROS_WARN ("Database - Save FAIL - NULL");

    /** Three cases: db_ptr_ == NULL -> return false
      * db_ptr_ != NULL, save success -> return db_ptr != NULL => return true
      * db_ptr_ != NULL, save fail -> return false
    **/
    return database_ptr_ != NULL;
}
