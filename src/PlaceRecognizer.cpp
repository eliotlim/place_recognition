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
#include <iostream>

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
    databaseChanged = false;

    // Initialize ORB Feature Detector and Extractors
    detector_ptr_ = cv::Ptr<cv::FeatureDetector> (new cv::ORB(1000));
    extractor_ptr_ = cv::Ptr<cv::DescriptorExtractor> (new cv::ORB(1000));
    output_descriptors_ = std::vector<cv::Mat>();

    ROS_INFO ("Place Recognizer Setup OK");
}

PlaceRecognizer::~PlaceRecognizer() {}

// Callback for Insertion Trigger
void PlaceRecognizer::trigger_callback (const std_msgs::String &msg) {
    // Insert descriptor features into Database
    ROS_INFO ("DB Insertion TRIGGERED");
    DBoW2::EntryId entryID = database_ptr_->add (output_descriptors_);
    ROS_INFO ("DB - Conversion to BoW Vector - OK");
    ROS_INFO ("DB Inserted EntryID: %u", entryID);
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

    /** Identify and Extract Features using ORB
      *
    **/
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detector_ptr_->detect (image_ptr_->image, keypoints);
    ROS_DEBUG ("Detected %lu keypoints", keypoints.size());
    // Draw Keypoints over output_ptr_
    cv::drawKeypoints (image_ptr_->image, keypoints, output_ptr_->image, cv::DrawMatchesFlags::DEFAULT);
    // Extract Descriptors from Keypoints
    ROS_DEBUG ("Extracting KeyPoints");
    extractor_ptr_->compute (image_ptr_->image, keypoints, descriptors);
    ROS_DEBUG ("computed cv::Mat output_descriptors_ size: <%d, %d>", descriptors.rows, descriptors.cols);
    transform_store_descriptors (descriptors);

    // TODO: Perform Database Query for matching images
    if (database_ptr_->size() > 0) {
        ROS_DEBUG ("DB - Querying");
        static DBoW2::QueryResults results;
        database_ptr_->query (output_descriptors_, results, 3);
        ROS_DEBUG ("DB - Parsing Results");
        DBoW2::Result top_result = *(results.begin());
        ROS_INFO ("Top Result: Image %u - Score: %f", top_result.Id, top_result.Score);
    }

    // Publish the augmented image
    output_pub_.publish (output_ptr_->toImageMsg());
}

// Load Vocabulary : Returns true if success, false if fail
bool PlaceRecognizer::loadVocabulary (const std::string& voc_filename) {

    // branching factor and depth levels - Adapted from DBoW2 Demo
    const int k = 12;
    const int L = 3;
    const DBoW2::WeightingType weight = DBoW2::TF_IDF;
    const DBoW2::ScoringType score = DBoW2::L1_NORM;

    vocabulary_ptr_ = boost::shared_ptr<ORBVocabulary> (new ORBVocabulary (k, L, weight, score));

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
bool PlaceRecognizer::loadDatabase (const std::string& db_filename) {
    bool fail = true;

    // Attempt to load the feature database from disk first.
    try {
        ROS_DEBUG ("Database - Loading - file: %s", db_filename.c_str());
        database_ptr_ = boost::shared_ptr<ORBDatabase> (new ORBDatabase (db_filename));
        fail = false;
        ROS_INFO ("Database - Load SUCCESS");
    } catch (std::string &exception_msg) {
        ROS_WARN ("Database - Load FAIL - file: %s : %s", db_filename.c_str(), exception_msg.c_str());
    } catch (cv::Exception &exception) {
        ROS_WARN ("Database - Load FAIL - file: %s : %s", db_filename.c_str(), exception.what());
    }

    // If feature database on disk is invalid, load vocabulary and create database.
    if (fail) {
        loadVocabulary();
        database_ptr_ = boost::shared_ptr<ORBDatabase> (new ORBDatabase (*vocabulary_ptr_, false, 0));
        fail = false;
        databaseChanged = true;
        ROS_INFO ("Database - Create SUCCESS");
    }

    return !fail;
}

// Save Database: Returns true if success, false if fail
bool PlaceRecognizer::saveDatabase (const std::string& db_filename) {

    // Attempt to save database to disk
    ROS_DEBUG ("Database - Saving");
    // Ignore if Database had no changes
    if (!databaseChanged) return false;

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

void PlaceRecognizer::transform_store_descriptors (const cv::Mat& mat) {
    // CV_8UC1 matrix only
    if (mat.type() == 0) {
        ROS_DEBUG ("Transform - U8C1 cv::Mat to vector<cv::Mat>");
        ROS_DEBUG ("cv::Mat descriptors type: %i size: <%d, %d>", mat.type(), mat.rows, mat.cols);
        output_descriptors_.clear();
        for (int row = 0; row < mat.rows; row++) {
            cv::Rect regionOfInterest (0, row, mat.cols, 1);
            output_descriptors_.push_back (cv::Mat (mat, regionOfInterest));
        }
        ROS_DEBUG ("Transform - SUCCESS");
    }
}
