// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/distortion_models.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/calib3d/calib3d.hpp>

#include <mynt_eye_ros_wrapper/GetInfo.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <map>
#include <string>

#include "mynteye/logger.h"
#include "mynteye/device/context.h"
#include "mynteye/device/device.h"
#define CONFIGURU_IMPLEMENTATION 1
#include "configuru.hpp"
using namespace configuru;  // NOLINT

#define PIE 3.1416
#define MATCH_CHECK_THRESHOLD 3

#define FULL_PRECISION \
  std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)

static const std::size_t MAXSIZE = 4;

MYNTEYE_BEGIN_NAMESPACE

namespace enc = sensor_msgs::image_encodings;
inline double compute_time(const double end, const double start) {
  return end - start;
}

class ROSWrapperNodelet : public nodelet::Nodelet {
 public:
  ROSWrapperNodelet() :
  skip_tag(-1),
  skip_tmp_left_tag(0),
  skip_tmp_right_tag(0) {
    unit_hard_time *= 10;
  }

  ~ROSWrapperNodelet() {
    // std::cout << __func__ << std::endl;
    if (device_) {
      device_->Stop(Source::ALL);
    }
    if (time_beg_ != -1) {
      double time_end = ros::Time::now().toSec();

      LOG(INFO) << "Time elapsed: " << compute_time(time_end, time_beg_)
                << " s";
      if (left_time_beg_ != -1) {
        LOG(INFO) << "Left count: " << left_count_ << ", fps: "
                  << (left_count_ / compute_time(time_end, left_time_beg_));
      }
      if (right_time_beg_ != -1) {
        LOG(INFO) << "Right count: " << right_count_ << ", fps: "
                  << (right_count_ / compute_time(time_end, right_time_beg_));
      }
      if (imu_time_beg_ != -1) {
          if (model_ == Model::STANDARD) {
            LOG(INFO) << "Imu count: " << imu_count_ << ", hz: "
                      << (imu_count_ /
                      compute_time(time_end, imu_time_beg_));
          } else {
          if (publish_imu_by_sync_) {
            LOG(INFO) << "imu_sync_count: " << imu_sync_count_ << ", hz: "
                      << (imu_sync_count_ /
                      compute_time(time_end, imu_time_beg_));
          } else {
            LOG(INFO) << "Imu count: " << imu_count_ << ", hz: "
                      << (imu_count_ /
                      compute_time(time_end, imu_time_beg_));
          }
        }
      }

      // ROS messages could not be reliably printed here, using glog instead :(
      // ros::Duration(1).sleep();  // 1s
      // https://answers.ros.org/question/35163/how-to-perform-an-action-at-nodelet-unload-shutdown/
    }
  }

  ros::Time hardTimeToSoftTime(std::uint64_t _hard_time) {
    static bool isInited = false;
    static double soft_time_begin(0);
    static std::uint64_t hard_time_begin(0);

    if (false == isInited) {
      soft_time_begin = ros::Time::now().toSec();
      hard_time_begin = _hard_time;
      isInited = true;
    }

    std::uint64_t time_ns_detal = (_hard_time - hard_time_begin);
    std::uint64_t time_ns_detal_s = time_ns_detal / 1000000;
    std::uint64_t time_ns_detal_ns = time_ns_detal % 1000000;
    double time_sec_double =
      ros::Time(time_ns_detal_s, time_ns_detal_ns * 1000).toSec();

    return ros::Time(soft_time_begin + time_sec_double);
  }

  // ros::Time hardTimeToSoftTime(std::uint64_t _hard_time) {
  //   static bool isInited = false;
  //   static double soft_time_begin(0);
  //   static std::uint64_t hard_time_begin(0);

  //   if (false == isInited) {
  //     soft_time_begin = ros::Time::now().toSec();
  //     hard_time_begin = _hard_time;
  //     isInited = true;
  //   }

  //   return ros::Time(
  //       static_cast<double>(soft_time_begin +
  //       static_cast<double>(_hard_time - hard_time_begin) * 0.000001f));
  // }

  inline bool is_overflow(std::uint64_t now,
      std::uint64_t pre) {

    return (now < pre) && ((pre - now) > (unit_hard_time / 2));
  }

  inline bool is_repeated(std::uint64_t now,
      std::uint64_t pre) {
    return now == pre;
  }

  inline bool is_abnormal(std::uint32_t now,
      std::uint32_t pre) {

    return (now < pre) && ((pre - now) < (unit_hard_time / 4));
  }

  ros::Time checkUpTimeStamp(std::uint64_t _hard_time,
      const Stream &stream) {
    static std::map<Stream, std::uint64_t> hard_time_now;
    static std::map<Stream, std::uint64_t> acc;

    if (is_overflow(_hard_time, hard_time_now[stream])) {
      acc[stream]++;
    }

    hard_time_now[stream] = _hard_time;

    return hardTimeToSoftTime(
        acc[stream] * unit_hard_time + _hard_time);
  }

  ros::Time checkUpImuTimeStamp(std::uint64_t _hard_time) {
    static std::uint64_t hard_time_now(0), acc(0);

    if (is_overflow(_hard_time, hard_time_now)) {
      acc++;
    }

    hard_time_now = _hard_time;

    return hardTimeToSoftTime(
        acc * unit_hard_time + _hard_time);
  }

  void onInit() override {
    nh_ = getMTNodeHandle();
    private_nh_ = getMTPrivateNodeHandle();

    initDevice();
    NODELET_FATAL_COND(device_ == nullptr, "No MYNT EYE device selected :(");

    pthread_mutex_init(&mutex_data_, nullptr);

    // node params

    std::map<Stream, std::string> stream_names{
        {Stream::LEFT, "left"},
        {Stream::RIGHT, "right"}
        };

    std::map<Stream, std::string> stream_topics{};
    for (auto &&it = stream_names.begin(); it != stream_names.end(); ++it) {
      stream_topics[it->first] = it->second;
      private_nh_.getParamCached(it->second + "_topic", stream_topics[it->first]);

      // if published init
      is_published_[it->first] = false;
    }
    is_motion_published_ = false;
    is_started_ = false;

    std::string imu_topic = "imu";
    std::string temperature_topic = "temperature";
    private_nh_.getParamCached("imu_topic", imu_topic);
    private_nh_.getParamCached("temperature_topic", temperature_topic);

    base_frame_id_ = "camera_link";
    private_nh_.getParamCached("base_frame_id", base_frame_id_);

    for (auto &&it = stream_names.begin(); it != stream_names.end(); ++it) {
      frame_ids_[it->first] = "camera_" + it->second + "_frame";
      private_nh_.getParamCached(it->second + "_frame_id", frame_ids_[it->first]);
    }

    imu_frame_id_ = "camera_imu_frame";
    temperature_frame_id_ = "camera_temperature_frame";
    private_nh_.getParamCached("imu_frame_id", imu_frame_id_);
    private_nh_.getParamCached("temperature_frame_id", temperature_frame_id_);

    gravity_ = 9.8;
    private_nh_.getParamCached("gravity", gravity_);

    // device options of standard210a
    if (model_ == Model::STANDARD210A) {
      option_names_ = {
          {Option::BRIGHTNESS, "standard210a/brightness"},
          {Option::EXPOSURE_MODE, "standard210a/exposure_mode"},
          {Option::MAX_GAIN, "standard210a/max_gain"},
          {Option::MAX_EXPOSURE_TIME, "standard210a/max_exposure_time"},
          {Option::DESIRED_BRIGHTNESS, "standard210a/desired_brightness"},
          {Option::MIN_EXPOSURE_TIME, "standard210a/min_exposure_time"},
          {Option::ACCELEROMETER_RANGE, "standard210a/accel_range"},
          {Option::GYROSCOPE_RANGE, "standard210a/gyro_range"},
          {Option::ACCELEROMETER_LOW_PASS_FILTER,
                  "standard210a/accel_low_filter"},
          {Option::GYROSCOPE_LOW_PASS_FILTER, "standard210a/gyro_low_filter"},
          {Option::IIC_ADDRESS_SETTING, "standard210a/iic_address_setting"}};
    }

    // device options of standard2
    if (model_ == Model::STANDARD2) {
      option_names_ = {
          {Option::BRIGHTNESS, "standard2/brightness"},
          {Option::EXPOSURE_MODE, "standard2/exposure_mode"},
          {Option::MAX_GAIN, "standard2/max_gain"},
          {Option::MAX_EXPOSURE_TIME, "standard2/max_exposure_time"},
          {Option::DESIRED_BRIGHTNESS, "standard2/desired_brightness"},
          {Option::MIN_EXPOSURE_TIME, "standard2/min_exposure_time"},
          {Option::IR_CONTROL, "STANDARD/ir_control"},
          {Option::ACCELEROMETER_RANGE, "standard2/accel_range"},
          {Option::GYROSCOPE_RANGE, "standard2/gyro_range"},
          {Option::ACCELEROMETER_LOW_PASS_FILTER, "standard2/accel_low_filter"},
          {Option::GYROSCOPE_LOW_PASS_FILTER, "standard2/gyro_low_filter"}};
    }
    // device options of standard
    if (model_ == Model::STANDARD) {
      option_names_ = {
          {Option::GAIN, "standard/gain"},
          {Option::BRIGHTNESS, "standard/brightness"},
          {Option::CONTRAST, "standard/contrast"},
          {Option::FRAME_RATE, "standard/frame_rate"},
          {Option::IMU_FREQUENCY, "standard/imu_frequency"},
          {Option::EXPOSURE_MODE, "standard/exposure_mode"},
          {Option::MAX_GAIN, "standard/max_gain"},
          {Option::MAX_EXPOSURE_TIME, "standard/max_exposure_time"},
          {Option::DESIRED_BRIGHTNESS, "standard/desired_brightness"},
          {Option::IR_CONTROL, "standard/ir_control"},
          {Option::HDR_MODE, "standard/hdr_mode"},
          {Option::ACCELEROMETER_RANGE, "standard/accel_range"},
          {Option::GYROSCOPE_RANGE, "standard/gyro_range"}};
    }

    // device options of standard200b
    if (model_ == Model::STANDARD200B) {
    option_names_ = {
        {Option::BRIGHTNESS, "standard200b/brightness"},
        {Option::EXPOSURE_MODE, "standard200b/exposure_mode"},
        {Option::MAX_GAIN, "standard200b/max_gain"},
        {Option::MAX_EXPOSURE_TIME, "standard200b/max_exposure_time"},
        {Option::DESIRED_BRIGHTNESS, "standard200b/desired_brightness"},
        {Option::MIN_EXPOSURE_TIME, "standard200b/min_exposure_time"},
        {Option::ACCELEROMETER_RANGE, "standard200b/accel_range"},
        {Option::GYROSCOPE_RANGE, "standard200b/gyro_range"},
        {Option::ACCELEROMETER_LOW_PASS_FILTER, "standard200b/accel_low_filter"},
        {Option::GYROSCOPE_LOW_PASS_FILTER, "standard200b/gyro_low_filter"}};
    }

    for (auto &&it = option_names_.begin(); it != option_names_.end(); ++it) {
      if (!device_->Supports(it->first))
        continue;
      int value = -1;
      private_nh_.getParamCached(it->second, value);
      if (value != -1) {
        NODELET_INFO_STREAM("Set " << it->second << " to " << value);
        device_->SetOptionValue(it->first, value);
      }
      NODELET_INFO_STREAM(it->first << ": " << device_->GetOptionValue(it->first));
    }

    // publishers

    image_transport::ImageTransport it_mynteye(nh_);

    for (auto &&it = stream_names.begin(); it != stream_names.end(); ++it) {
      auto &&topic = stream_topics[it->first];
      // camera
      camera_publishers_[it->first] = it_mynteye.advertise(topic, 1);
      NODELET_INFO_STREAM("Advertized on topic " << topic);
    }

    if (model_ == Model::STANDARD2 ||
        model_ == Model::STANDARD210A || model_ == Model::STANDARD200B) {
      camera_encodings_ = {{Stream::LEFT, enc::BGR8},
                          {Stream::RIGHT, enc::BGR8}};
    }
    if (model_ == Model::STANDARD) {
      camera_encodings_ = {{Stream::LEFT, enc::MONO8},
        {Stream::RIGHT, enc::MONO8}};
    }
    pub_imu_ = nh_.advertise<sensor_msgs::Imu>(imu_topic, 100);
    NODELET_INFO_STREAM("Advertized on topic " << imu_topic);

    pub_temperature_ = nh_.advertise<
                        sensor_msgs::Temperature>(temperature_topic, 100);
    NODELET_INFO_STREAM("Advertized on topic " << temperature_topic);


    int ros_output_framerate = -1;
    private_nh_.getParamCached("ros_output_framerate_cut", ros_output_framerate);
    if (ros_output_framerate > 0 && ros_output_framerate < 7) {
      skip_tag = ros_output_framerate;
    }

    // services

    const std::string DEVICE_INFO_SERVICE = "get_info";
    get_info_service_ = nh_.advertiseService(
        DEVICE_INFO_SERVICE, &ROSWrapperNodelet::getInfo, this);
    NODELET_INFO_STREAM("Advertized service " << DEVICE_INFO_SERVICE);
    publishStaticTransforms();
    ros::Rate loop_rate(frame_rate_);
    while (private_nh_.ok()) {
      publishTopics();
      loop_rate.sleep();
    }
  }

  bool getInfo(
      mynt_eye_ros_wrapper::GetInfo::Request &req,     // NOLINT
      mynt_eye_ros_wrapper::GetInfo::Response &res) {  // NOLINT
    using Request = mynt_eye_ros_wrapper::GetInfo::Request;
    switch (req.key) {
      case Request::DEVICE_NAME:
        res.value = device_->GetInfo(Info::DEVICE_NAME);
        break;
      case Request::SERIAL_NUMBER:
        res.value = device_->GetInfo(Info::SERIAL_NUMBER);
        break;
      case Request::FIRMWARE_VERSION:
        res.value = device_->GetInfo(Info::FIRMWARE_VERSION);
        break;
      case Request::HARDWARE_VERSION:
        res.value = device_->GetInfo(Info::HARDWARE_VERSION);
        break;
      case Request::SPEC_VERSION:
        res.value = device_->GetInfo(Info::SPEC_VERSION);
        break;
      case Request::LENS_TYPE:
        res.value = device_->GetInfo(Info::LENS_TYPE);
        break;
      case Request::IMU_TYPE:
        res.value = device_->GetInfo(Info::IMU_TYPE);
        break;
      case Request::NOMINAL_BASELINE:
        res.value = device_->GetInfo(Info::NOMINAL_BASELINE);
        break;
      case Request::AUXILIARY_CHIP_VERSION:
        res.value = device_->GetInfo(Info::AUXILIARY_CHIP_VERSION);
        break;
      case Request::ISP_VERSION:
        res.value = device_->GetInfo(Info::ISP_VERSION);
        break;
      case Request::IMG_EXTRINSICS_RTOL:
      {
        auto extri = device_->GetExtrinsics(Stream::RIGHT, Stream::LEFT);
        Config extrinsics{
          {"rotation",     Config::array({extri.rotation[0][0], extri.rotation[0][1], extri.rotation[0][2],   // NOLINT
                                          extri.rotation[1][0], extri.rotation[1][1], extri.rotation[1][2],   // NOLINT
                                          extri.rotation[2][0], extri.rotation[2][1], extri.rotation[2][2]})},// NOLINT
          {"translation",  Config::array({extri.translation[0], extri.translation[1], extri.translation[2]})} // NOLINT
        };
        std::string json = dump_string(extrinsics, configuru::JSON);
        res.value = json;
      }
      break;
      case Request::IMU_INTRINSICS:
      {
        bool is_ok;
        auto intri = device_->GetMotionIntrinsics();
        Config intrinsics {
          {"accel", {
            {"scale",     Config::array({ intri.accel.scale[0][0], intri.accel.scale[0][1],  intri.accel.scale[0][2],   // NOLINT
                                          intri.accel.scale[1][0], intri.accel.scale[1][1],  intri.accel.scale[1][2],   // NOLINT
                                          intri.accel.scale[2][0], intri.accel.scale[2][1],  intri.accel.scale[2][2]})},// NOLINT
            {"drift",     Config::array({ intri.accel.drift[0],    intri.accel.drift[1],     intri.accel.drift[2]})}, // NOLINT
            {"noise",     Config::array({ intri.accel.noise[0],    intri.accel.noise[1],     intri.accel.noise[2]})}, // NOLINT
            {"bias",      Config::array({ intri.accel.bias[0],     intri.accel.bias[1],      intri.accel.bias[2]})} // NOLINT
          }},
          {"gyro", {
            {"scale",     Config::array({ intri.gyro.scale[0][0], intri.gyro.scale[0][1],  intri.gyro.scale[0][2],   // NOLINT
                                          intri.gyro.scale[1][0], intri.gyro.scale[1][1],  intri.gyro.scale[1][2],   // NOLINT
                                          intri.gyro.scale[2][0], intri.gyro.scale[2][1],  intri.gyro.scale[2][2]})},// NOLINT
            {"drift",     Config::array({ intri.gyro.drift[0],    intri.gyro.drift[1],     intri.gyro.drift[2]})}, // NOLINT
            {"noise",     Config::array({ intri.gyro.noise[0],    intri.gyro.noise[1],     intri.gyro.noise[2]})}, // NOLINT
            {"bias",      Config::array({ intri.gyro.bias[0],     intri.gyro.bias[1],      intri.gyro.bias[2]})} // NOLINT
          }}
        };
        std::string json = dump_string(intrinsics, JSON);
        res.value = json;
      }
      break;
      case Request::IMU_EXTRINSICS:
      {
        auto extri = device_->GetMotionExtrinsics(Stream::LEFT);
        Config extrinsics{
          {"rotation",     Config::array({extri.rotation[0][0], extri.rotation[0][1], extri.rotation[0][2],   // NOLINT
                                          extri.rotation[1][0], extri.rotation[1][1], extri.rotation[1][2],   // NOLINT
                                          extri.rotation[2][0], extri.rotation[2][1], extri.rotation[2][2]})},// NOLINT
          {"translation",  Config::array({extri.translation[0], extri.translation[1], extri.translation[2]})} // NOLINT
        };
        std::string json = dump_string(extrinsics, configuru::JSON);
        res.value = json;
      }
      break;
      default:
        NODELET_WARN_STREAM("Info of key " << req.key << " not exist");
        return false;
    }
    return true;
  }

  int getStreamSubscribers(const Stream &stream) {
    auto pub = camera_publishers_[stream];
    if (pub)
      return pub.getNumSubscribers();
    return -1;
  }


  void publishTopics() {
    if (camera_publishers_[Stream::LEFT].getNumSubscribers() > 0 &&
        !is_published_[Stream::LEFT]) {
        NODELET_DEBUG_STREAM("interrupt1!!!");
      	device_->SetStreamCallback(
          Stream::LEFT, [&](const device::StreamData &data) {
            ++left_count_;
            if (left_count_ > 10) {
              // ros::Time stamp = hardTimeToSoftTime(data.img->timestamp);
              ros::Time stamp = checkUpTimeStamp(
                  data.img->timestamp, Stream::LEFT);
              if (skip_tag > 0) {
                if (skip_tmp_left_tag == 0) {
                  skip_tmp_left_tag = skip_tag;
                } else {
                  skip_tmp_left_tag--;
                  return;
                }
                if (left_timestamps.size() < MATCH_CHECK_THRESHOLD) {
                  left_timestamps.insert(left_timestamps.begin(), data.img->timestamp);
                } else {
                  left_timestamps.insert(left_timestamps.begin(), data.img->timestamp);
                  left_timestamps.pop_back();
                }
              }
              publishCamera(Stream::LEFT, data, left_count_, stamp);
              NODELET_DEBUG_STREAM(
                  Stream::LEFT << ", count: " << left_count_
                      << ", frame_id: " << data.img->frame_id
                      << ", timestamp: " << data.img->timestamp
                      << ", is_ets: " << std::boolalpha << data.img->is_ets
                      << ", exposure_time: " << data.img->exposure_time);
            }
          });
      left_time_beg_ = ros::Time::now().toSec();
      is_published_[Stream::LEFT] = true;
    }

    if (camera_publishers_[Stream::RIGHT].getNumSubscribers() > 0 &&
        !is_published_[Stream::RIGHT]) {
      device_->SetStreamCallback(
          Stream::RIGHT, [&](const device::StreamData &data) {
            ++right_count_;
            if (right_count_ > 10) {
              // ros::Time stamp = hardTimeToSoftTime(data.img->timestamp);
              ros::Time stamp = checkUpTimeStamp(
                  data.img->timestamp, Stream::RIGHT);
              if (skip_tag > 0) {
                if (skip_tmp_right_tag == 0) {
                  skip_tmp_right_tag = skip_tag;
                } else {
                  skip_tmp_right_tag--;
                  return;
                }
                if (right_timestamps.size() < MATCH_CHECK_THRESHOLD) {
                  right_timestamps.insert(right_timestamps.begin(), data.img->timestamp);
                } else {
                  right_timestamps.insert(right_timestamps.begin(), data.img->timestamp);
                  right_timestamps.pop_back();
                  bool is_match = false;
                  for (size_t i = 0; i < right_timestamps.size(); i++) {
                    for (size_t j = 0; j < left_timestamps.size(); j++) {
                      if (right_timestamps[i] == left_timestamps[j]) {
                        is_match = true;
                        break;
                      }
                    }
                    if (is_match) {
                      break;
                    }
                  }

                  if (!is_match) {
                    std::cout << "find the output stamp can't matched try to fix with one skip step." << std::endl;
                    skip_tmp_right_tag++;
                  }
                }
              }
              publishCamera(Stream::RIGHT, data, right_count_, stamp);
              NODELET_DEBUG_STREAM(
                  Stream::RIGHT << ", count: " << right_count_
                      << ", frame_id: " << data.img->frame_id
                      << ", timestamp: " << data.img->timestamp
                      << ", is_ets: " << std::boolalpha << data.img->is_ets
                      << ", exposure_time: " << data.img->exposure_time);
            }
          });
      right_time_beg_ = ros::Time::now().toSec();
      is_published_[Stream::RIGHT] = true;
    }

    if (!is_motion_published_) {
      device_->SetMotionCallback([this](const device::MotionData &data) {
      ros::Time stamp = checkUpImuTimeStamp(data.imu->timestamp);

      // static double imu_time_prev = -1;
      // NODELET_INFO_STREAM("ros_time_beg: " << FULL_PRECISION << ros_time_beg
      //     << ", imu_time_elapsed: " << FULL_PRECISION
      //     << ((data.imu->timestamp - imu_time_beg) * 0.00001f)
      //     << ", imu_time_diff: " << FULL_PRECISION
      //     << ((imu_time_prev < 0) ? 0
      //         : (data.imu->timestamp - imu_time_prev) * 0.01f) << " ms");
      // imu_time_prev = data.imu->timestamp;
      ++imu_count_;
      if (imu_count_ > 50) {
        if (publish_imu_by_sync_) {
          if (data.imu) {
            if (data.imu->flag == 1) {  // accelerometer
              imu_accel_ = data.imu;
              publishImuBySync();
            } else if (data.imu->flag == 2) {  // gyroscope
              imu_gyro_ = data.imu;
              publishImuBySync();
            } else {
              publishImu(data, imu_count_, stamp);
              publishTemperature(data.imu->temperature, imu_count_, stamp);
            }
          } else {
            NODELET_WARN_STREAM("Motion data is empty");
          }
        } else {
          publishImu(data, imu_count_, stamp);
          publishTemperature(data.imu->temperature, imu_count_, stamp);
        }
        NODELET_DEBUG_STREAM(
            "Imu count: " << imu_count_
                          << ", timestamp: " << data.imu->timestamp
                          << ", is_ets: " << std::boolalpha << data.imu->is_ets
                          << ", accel_x: " << data.imu->accel[0]
                          << ", accel_y: " << data.imu->accel[1]
                          << ", accel_z: " << data.imu->accel[2]
                          << ", gyro_x: " << data.imu->gyro[0]
                          << ", gyro_y: " << data.imu->gyro[1]
                          << ", gyro_z: " << data.imu->gyro[2]
                          << ", temperature: " << data.imu->temperature);
        // Sleep 1ms, otherwise publish may drop some datas.
        ros::Duration(0.001).sleep();
      }
      });
      imu_time_beg_ = ros::Time::now().toSec();
      is_motion_published_ = true;
    }

    if (!is_started_) {
      time_beg_ = ros::Time::now().toSec();
      device_->Start(Source::ALL);
      is_started_ = true;
    }
  }

  void publishCamera(
      const Stream &stream, const device::StreamData &data, std::uint32_t seq,
      ros::Time stamp) {


    std_msgs::Header header;
    header.seq = seq;
    header.stamp = stamp;
    header.frame_id = frame_ids_[stream];
    pthread_mutex_lock(&mutex_data_);
    auto data_frame = *(data.frame);

    cv::Mat img;
    if (data_frame.format() == Format::GREY) {
      cv::Mat temp_img(
          data_frame.height(), data_frame.width(), CV_8UC1,
          data_frame.data());
      img = temp_img;
    } else if (data_frame.format() == Format::YUYV) {
      cv::Mat temp_img(
          data_frame.height(), data_frame.width(), CV_8UC2,
          data_frame.data());
      cv::cvtColor(temp_img, temp_img, cv::COLOR_YUV2BGR_YUY2);
      img = temp_img;
    } else if (data_frame.format() == Format::BGR888) {
      cv::Mat temp_img(
          data_frame.height(), data_frame.width(), CV_8UC3,
          data_frame.data());
      img = temp_img;
    }

    auto &&msg =
        cv_bridge::CvImage(header, camera_encodings_[stream], img).toImageMsg();
    pthread_mutex_unlock(&mutex_data_);
    camera_publishers_[stream].publish(msg);
  }


  void publishImu(
      const device::MotionData &data, std::uint32_t seq, ros::Time stamp) {
    if (pub_imu_.getNumSubscribers() == 0)
      return;

    sensor_msgs::Imu msg;

    msg.header.seq = seq;
    msg.header.stamp = stamp;
    msg.header.frame_id = imu_frame_id_;

    // acceleration should be in m/s^2 (not in g's)
    msg.linear_acceleration.x = data.imu->accel[0] * gravity_;
    msg.linear_acceleration.y = data.imu->accel[1] * gravity_;
    msg.linear_acceleration.z = data.imu->accel[2] * gravity_;

    msg.linear_acceleration_covariance[0] = 0;
    msg.linear_acceleration_covariance[1] = 0;
    msg.linear_acceleration_covariance[2] = 0;

    msg.linear_acceleration_covariance[3] = 0;
    msg.linear_acceleration_covariance[4] = 0;
    msg.linear_acceleration_covariance[5] = 0;

    msg.linear_acceleration_covariance[6] = 0;
    msg.linear_acceleration_covariance[7] = 0;
    msg.linear_acceleration_covariance[8] = 0;

    // velocity should be in rad/sec
    msg.angular_velocity.x = data.imu->gyro[0] * M_PI / 180;
    msg.angular_velocity.y = data.imu->gyro[1] * M_PI / 180;
    msg.angular_velocity.z = data.imu->gyro[2] * M_PI / 180;

    msg.angular_velocity_covariance[0] = 0;
    msg.angular_velocity_covariance[1] = 0;
    msg.angular_velocity_covariance[2] = 0;

    msg.angular_velocity_covariance[3] = 0;
    msg.angular_velocity_covariance[4] = 0;
    msg.angular_velocity_covariance[5] = 0;

    msg.angular_velocity_covariance[6] = 0;
    msg.angular_velocity_covariance[7] = 0;
    msg.angular_velocity_covariance[8] = 0;

    pub_imu_.publish(msg);
  }

  void timestampAlign() {
    static std::vector<ImuData> acc_buf;
    static std::vector<ImuData> gyro_buf;

    if (imu_accel_ != nullptr) {
      acc_buf.push_back(*imu_accel_);
    }

    if (imu_gyro_ != nullptr) {
      gyro_buf.push_back(*imu_gyro_);
    }

    imu_accel_ = nullptr;
    imu_gyro_ = nullptr;

    imu_align_.clear();

    if (acc_buf.empty() || gyro_buf.empty()) {
      return;
    }

    ImuData imu_temp;
    auto itg = gyro_buf.end();
    auto ita = acc_buf.end();
    for (auto it_gyro = gyro_buf.begin();
        it_gyro != gyro_buf.end(); it_gyro++) {
      for (auto it_acc = acc_buf.begin();
          it_acc+1 != acc_buf.end(); it_acc++) {
        if (it_gyro->timestamp >= it_acc->timestamp
            && it_gyro->timestamp <= (it_acc+1)->timestamp) {
          double k = static_cast<double>((it_acc+1)->timestamp - it_acc->timestamp);
          k = static_cast<double>(it_gyro->timestamp - it_acc->timestamp) / k;

          imu_temp = *it_gyro;
          imu_temp.accel[0] = it_acc->accel[0] + ((it_acc+1)->accel[0] - it_acc->accel[0]) * k;
          imu_temp.accel[1] = it_acc->accel[1] + ((it_acc+1)->accel[1] - it_acc->accel[1]) * k;
          imu_temp.accel[2] = it_acc->accel[2] + ((it_acc+1)->accel[2] - it_acc->accel[2]) * k;

          imu_align_.push_back(imu_temp);

          itg = it_gyro;
          ita = it_acc;
        }
      }
    }

    if (itg != gyro_buf.end()) {
      gyro_buf.erase(gyro_buf.begin(), itg + 1);
    }

    if (ita != acc_buf.end()) {
      acc_buf.erase(acc_buf.begin(), ita);
    }
  }

  void publishImuBySync() {
    timestampAlign();

    for (int i = 0; i < imu_align_.size(); i++) {
      sensor_msgs::Imu msg;

      msg.header.seq = imu_sync_count_;
      ros::Time stamp = checkUpImuTimeStamp(imu_align_[i].timestamp);
      msg.header.stamp = stamp;
      msg.header.frame_id = imu_frame_id_;

      // acceleration should be in m/s^2 (not in g's)
      msg.linear_acceleration.x = imu_align_[i].accel[0] * gravity_;
      msg.linear_acceleration.y = imu_align_[i].accel[1] * gravity_;
      msg.linear_acceleration.z = imu_align_[i].accel[2] * gravity_;

      msg.linear_acceleration_covariance[0] = 0;
      msg.linear_acceleration_covariance[1] = 0;
      msg.linear_acceleration_covariance[2] = 0;

      msg.linear_acceleration_covariance[3] = 0;
      msg.linear_acceleration_covariance[4] = 0;
      msg.linear_acceleration_covariance[5] = 0;

      msg.linear_acceleration_covariance[6] = 0;
      msg.linear_acceleration_covariance[7] = 0;
      msg.linear_acceleration_covariance[8] = 0;

      // velocity should be in rad/sec
      msg.angular_velocity.x = imu_align_[i].gyro[0] * M_PI / 180;
      msg.angular_velocity.y = imu_align_[i].gyro[1] * M_PI / 180;
      msg.angular_velocity.z = imu_align_[i].gyro[2] * M_PI / 180;

      msg.angular_velocity_covariance[0] = 0;
      msg.angular_velocity_covariance[1] = 0;
      msg.angular_velocity_covariance[2] = 0;

      msg.angular_velocity_covariance[3] = 0;
      msg.angular_velocity_covariance[4] = 0;
      msg.angular_velocity_covariance[5] = 0;

      msg.angular_velocity_covariance[6] = 0;
      msg.angular_velocity_covariance[7] = 0;
      msg.angular_velocity_covariance[8] = 0;

      pub_imu_.publish(msg);

      publishTemperature(imu_align_[i].temperature, imu_sync_count_, stamp);

      ++imu_sync_count_;
    }
  }

  void publishTemperature(
    float temperature, std::uint32_t seq, ros::Time stamp) {
    if (pub_temperature_.getNumSubscribers() == 0)
      return;
    sensor_msgs::Temperature msg;
    msg.header.seq = seq;
    msg.header.stamp = stamp;
    msg.header.frame_id = temperature_frame_id_;
    msg.temperature = temperature;
    msg.variance = 0;
    pub_temperature_.publish(msg);
  }

 private:
  void initDevice() {
    std::shared_ptr<Device> device = nullptr;

    device = selectDevice();

    device_ = device;
    auto &&requests = device->GetStreamRequests();
    std::size_t m = requests.size();
    int request_index = 0;

    model_ = device_->GetModel();
    if (model_ == Model::STANDARD2 ||
        model_ == Model::STANDARD210A || model_ == Model::STANDARD200B) {
      private_nh_.getParamCached("standard2/request_index", request_index);
      switch (request_index) {
        case 0:
        case 4:
          frame_rate_ = 10;
          break;
        case 1:
        case 5:
          frame_rate_ = 20;
          break;
        case 2:
        case 6:
          frame_rate_ = 30;
          break;
        case 3:
          frame_rate_ = 60;
          break;
      }
    }
    if (model_ == Model::STANDARD) {
      private_nh_.getParamCached("standard/request_index", request_index);
      frame_rate_ = device_->GetOptionValue(Option::FRAME_RATE);
    }

    std::int32_t process_mode = 0;
    if (model_ == Model::STANDARD2 ||
        model_ == Model::STANDARD210A || model_ == Model::STANDARD200B) {
      private_nh_.getParamCached("standard2/imu_process_mode", process_mode);
      device_->EnableProcessMode(process_mode);
    }

    NODELET_FATAL_COND(m <= 0, "No MYNT EYE devices :(");
    if (m <= 1) {
      NODELET_INFO_STREAM("Only one stream request, select index: 0");
      device_->ConfigStreamRequest(requests[0]);
    } else {
      if (request_index >= m) {
        NODELET_WARN_STREAM("Resquest_index out of range");
        device_->ConfigStreamRequest(requests[0]);
      } else {
        device_->ConfigStreamRequest(requests[request_index]);
      }
    }
  }

  std::shared_ptr<Device> selectDevice() {
    NODELET_INFO_STREAM("Detecting MYNT EYE devices");

    Context context;
    auto &&devices = context.devices();

    size_t n = devices.size();
    NODELET_FATAL_COND(n <= 0, "No MYNT EYE devices :(");

    NODELET_INFO_STREAM("MYNT EYE devices:");
    for (size_t i = 0; i < n; i++) {
      auto &&device = devices[i];
      auto &&name = device->GetInfo(Info::DEVICE_NAME);
      auto &&serial_number = device->GetInfo(Info::SERIAL_NUMBER);
      NODELET_INFO_STREAM("  index: " << i << ", name: " <<
          name << ", serial number: " << serial_number);
    }

    bool is_multiple = false;
    private_nh_.getParam("is_multiple", is_multiple);
    if (is_multiple) {
      std::string sn;
      private_nh_.getParam("serial_number", sn);
      NODELET_FATAL_COND(sn.empty(), "Must set serial_number "
          "in mynteye_1.launch and mynteye_2.launch.");

      for (size_t i = 0; i < n; i++) {
        auto &&device = devices[i];
        auto &&name = device->GetInfo(Info::DEVICE_NAME);
        auto &&serial_number = device->GetInfo(Info::SERIAL_NUMBER);
        if (sn == serial_number)
          return device;
        NODELET_FATAL_COND(i == (n - 1), "No corresponding device was found,"
            " check the serial_number configuration. ");
      }
    } else {
      if (n <= 1) {
        NODELET_INFO_STREAM("Only one MYNT EYE device, select index: 0");
        return devices[0];
      } else {
        while (true) {
          size_t i;
          NODELET_INFO_STREAM(
              "There are " << n << " MYNT EYE devices, select index: ");
          std::cin >> i;
          if (i >= n) {
            NODELET_WARN_STREAM("Index out of range :(");
            continue;
          }
          return devices[i];
        }
      }
    }

    return nullptr;
  }

  std::shared_ptr<IntrinsicsBase> getDefaultIntrinsics() {
    auto res = std::make_shared<IntrinsicsPinhole>();
    res->width = 640;
    res->height = 400;
    res->model = 0;
    res->fx = 3.6220059643202876e+02;
    res->fy = 3.6350065250745848e+02;
    res->cx = 4.0658699068023441e+02;
    res->cy = 2.3435161110061483e+02;
    double codffs[5] = {
      -2.5034765682756088e-01,
      5.0579399202897619e-02,
      -7.0536676161976066e-04,
      -8.5255451307033846e-03,
      0.
    };
    for (unsigned int i = 0; i < 5; i++) {
      res->coeffs[i] = codffs[i];
    }
    return res;
  }

  std::shared_ptr<Extrinsics> getDefaultExtrinsics() {
    auto res = std::make_shared<Extrinsics>();
    double rotation[9] = {
      9.9867908939669447e-01,  -6.3445566137485428e-03, 5.0988459509619687e-02,
      5.9890316389333252e-03,  9.9995670037792639e-01,  7.1224201868366971e-03,
      -5.1031440326695092e-02, -6.8076406092671274e-03, 9.9867384471984544e-01
    };
    double translation[3] = {-1.2002489764113250e+02, -1.1782637409050747e+00,
        -5.2058205159996538e+00};
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 3; j++) {
        res->rotation[i][j] = rotation[i*3 + j];
      }
    }
    for (unsigned int i = 0; i < 3; i++) {
      res->translation[i] = translation[i];
    }
    return res;
  }

 void publishStaticTransforms() {
    ros::Time tf_stamp = ros::Time::now();

    // The left frame is used as the base frame.
    geometry_msgs::TransformStamped b2l_msg;
    b2l_msg.header.stamp = tf_stamp;
    b2l_msg.header.frame_id = base_frame_id_;
    b2l_msg.child_frame_id = frame_ids_[Stream::LEFT];
    b2l_msg.transform.translation.x = 0;
    b2l_msg.transform.translation.y = 0;
    b2l_msg.transform.translation.z = 0;
    b2l_msg.transform.rotation.x = 0;
    b2l_msg.transform.rotation.y = 0;
    b2l_msg.transform.rotation.z = 0;
    b2l_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2l_msg);

    // Transform left frame to right frame
    auto &&l2r_ex = device_->GetExtrinsics(Stream::LEFT, Stream::RIGHT);
    tf::Quaternion l2r_q;
    tf::Matrix3x3 l2r_r(
        l2r_ex.rotation[0][0], l2r_ex.rotation[0][1], l2r_ex.rotation[0][2],
        l2r_ex.rotation[1][0], l2r_ex.rotation[1][1], l2r_ex.rotation[1][2],
        l2r_ex.rotation[2][0], l2r_ex.rotation[2][1], l2r_ex.rotation[2][2]);
    l2r_r.getRotation(l2r_q);
    geometry_msgs::TransformStamped l2r_msg;
    l2r_msg.header.stamp = tf_stamp;
    l2r_msg.header.frame_id = frame_ids_[Stream::LEFT];
    l2r_msg.child_frame_id = frame_ids_[Stream::RIGHT];
    l2r_msg.transform.translation.x = l2r_ex.translation[0] / 1000;
    l2r_msg.transform.translation.y = l2r_ex.translation[1] / 1000;
    l2r_msg.transform.translation.z = l2r_ex.translation[2] / 1000;
    l2r_msg.transform.rotation.x = l2r_q.getX();
    l2r_msg.transform.rotation.y = l2r_q.getY();
    l2r_msg.transform.rotation.z = l2r_q.getZ();
    l2r_msg.transform.rotation.w = l2r_q.getW();
    static_tf_broadcaster_.sendTransform(l2r_msg);


    // Transform left frame to imu frame
    auto &&l2i_ex = device_->GetMotionExtrinsics(Stream::LEFT);
    geometry_msgs::TransformStamped l2i_msg;
    l2i_msg.header.stamp = tf_stamp;
    l2i_msg.header.frame_id = frame_ids_[Stream::LEFT];
    l2i_msg.child_frame_id = imu_frame_id_;
    bool is_data_use_mm_instead_of_m = abs(l2i_ex.translation[0]) > 1.0 ||
                                       abs(l2i_ex.translation[1]) > 1.0 ||
                                       abs(l2i_ex.translation[2]) > 1.0;
    if (is_data_use_mm_instead_of_m) {
      l2i_msg.transform.translation.x = l2i_ex.translation[0] * 0.001;
      l2i_msg.transform.translation.y = l2i_ex.translation[1] * 0.001;
      l2i_msg.transform.translation.z = l2i_ex.translation[2] * 0.001;
    } else {
      l2i_msg.transform.translation.x = l2i_ex.translation[0];
      l2i_msg.transform.translation.y = l2i_ex.translation[1];
      l2i_msg.transform.translation.z = l2i_ex.translation[2];
    }
    // LOG(INFO) << std::endl << "l2i_msg.transform.translation.x: "
    //           << l2i_msg.transform.translation.x << std::endl
    //           << "l2i_msg.transform.translation.y: "
    //           << l2i_msg.transform.translation.y << std::endl
    //           << "l2i_msg.transform.translation.z: "
    //           << l2i_msg.transform.translation.z << std::endl;
    if (l2i_ex.rotation[0][0] == 0 && l2i_ex.rotation[2][2] == 0) {
      l2i_msg.transform.rotation.x = 0;
      l2i_msg.transform.rotation.y = 0;
      l2i_msg.transform.rotation.z = 0;
      l2i_msg.transform.rotation.w = 1;
    } else {
      tf::Quaternion l2i_q;
      tf::Matrix3x3 l2i_r(
          l2i_ex.rotation[0][0], l2i_ex.rotation[0][1], l2i_ex.rotation[0][2],
          l2i_ex.rotation[1][0], l2i_ex.rotation[1][1], l2i_ex.rotation[1][2],
          l2i_ex.rotation[2][0], l2i_ex.rotation[2][1], l2i_ex.rotation[2][2]);
      l2i_r.getRotation(l2i_q);
      l2i_msg.transform.rotation.x = l2i_q.getX();
      l2i_msg.transform.rotation.y = l2i_q.getY();
      l2i_msg.transform.rotation.z = l2i_q.getZ();
      l2i_msg.transform.rotation.w = l2i_q.getW();
    }
    static_tf_broadcaster_.sendTransform(l2i_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  pthread_mutex_t mutex_data_;

  Model model_;
  std::map<Option, std::string> option_names_;
  // camera:
  //   LEFT, RIGHT, LEFT_RECTIFIED, RIGHT_RECTIFIED,
  //   DISPARITY, DISPARITY_NORMALIZED,
  //   DEPTH
  std::map<Stream, image_transport::Publisher> camera_publishers_;
  std::map<Stream, std::string> camera_encodings_;

  ros::Publisher pub_imu_;
  ros::Publisher pub_temperature_;

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  ros::ServiceServer get_info_service_;

  // node params

  std::string base_frame_id_;
  std::string imu_frame_id_;
  std::string temperature_frame_id_;
  std::map<Stream, std::string> frame_ids_;

  double gravity_;

  // api

  std::shared_ptr<Device> device_;

  double time_beg_ = -1;
  double left_time_beg_ = -1;
  double right_time_beg_ = -1;
  double imu_time_beg_ = -1;
  std::size_t left_count_ = 0;
  std::size_t right_count_ = 0;
  std::size_t imu_count_ = 0;
  std::size_t imu_sync_count_ = 0;
  std::shared_ptr<ImuData> imu_accel_;
  std::shared_ptr<ImuData> imu_gyro_;
  bool publish_imu_by_sync_ = true;
  std::map<Stream, bool> is_published_;
  bool is_motion_published_;
  bool is_started_;
  int frame_rate_;
  std::vector<ImuData> imu_align_;
  int skip_tag;
  int skip_tmp_left_tag;
  int skip_tmp_right_tag;
  std::vector<int64_t> left_timestamps;
  std::vector<int64_t> right_timestamps;

  std::uint64_t unit_hard_time = std::numeric_limits<std::uint32_t>::max();
};

MYNTEYE_END_NAMESPACE

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(mynteye::ROSWrapperNodelet, nodelet::Nodelet);
