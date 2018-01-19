/// @brief Detector class for yolo2
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 ThundeRatz
 * Copyright (C) 2017 Toyota Motor Corporation

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tmc_yolo2_ros/Detection.h>

#include "tmc_yolo2_ros/detector.hpp"

extern "C" {
#undef __cplusplus
#include <tmc_darknet/detection_layer.h>
#include <tmc_darknet/parser.h>
#include <tmc_darknet/region_layer.h>
#include <tmc_darknet/utils.h>
#define __cplusplus
}

namespace tmc_yolo2_ros {

/// 認識器の初期化を行う
bool Detector::Initialize(const std::string& config_path,
                          const std::string& weights_path,
                          const std::string& class_names_path,
                          double confidence_threshold,
                          double nms_threshold,
                          uint32_t& dst_width,
                          uint32_t& dst_height) {
  confidence_threshold_ = confidence_threshold;
  nms_threshold_ = nms_threshold;
  net_ = parse_network_cfg(const_cast<char*>(config_path.c_str()));
  dst_width = net_.w;
  dst_height = net_.h;

  load_weights(&net_, const_cast<char*>(weights_path.c_str()));
  set_batch_network(&net_, 1);

  std::ifstream ifs(class_names_path.c_str());
  std::string line;
  while (ifs && getline(ifs, line)) {
    class_names_.push_back(line);
  }
  layer output_layer = net_.layers[net_.n - 1];
  if (output_layer.type != DETECTION &&
      output_layer.type != REGION) {
    ROS_ERROR("Last layer is invalid.");
    return false;
  }
  if (output_layer.classes != class_names_.size()) {
    ROS_ERROR("Class sizes differ between network definition(%d) and class_names file(%d).",
              output_layer.classes, class_names_.size());
    return false;
  }


  boxes_.resize(output_layer.w * output_layer.h * output_layer.n);
  probs_.resize(output_layer.w * output_layer.h * output_layer.n);
  float* probs_mem = new float[probs_.size() * output_layer.classes];
  std::vector<float *>::iterator it = probs_.begin();
  for (; it != probs_.end(); ++it) {
    *it = probs_mem;
    probs_mem += output_layer.classes;
  }
  return true;
}

/// 認識処理を実行する
std::vector<tmc_yolo2_ros::Detection> Detector::Execute(image* darknet_image,
                                                        uint32_t image_w,
                                                        uint32_t image_h) {
  float *prediction = network_predict(net_, darknet_image->data);
  layer output_layer = net_.layers[net_.n - 1];
  output_layer.output = prediction;

  if (output_layer.type == DETECTION) {
    get_detection_boxes(output_layer, 1, 1, confidence_threshold_,
                        probs_.data(), boxes_.data(), 0);
  } else if (output_layer.type == REGION) {
    get_region_boxes(output_layer, 1, 1, confidence_threshold_,
                     probs_.data(), boxes_.data(), 0, 0, .5);
  }
  int num_classes = output_layer.classes;
  do_nms(boxes_.data(), probs_.data(),
         output_layer.w * output_layer.h * output_layer.n, num_classes, nms_threshold_);
  std::vector<tmc_yolo2_ros::Detection> detections;
  for (size_t i = 0; i < probs_.size(); i++) {
    int class_id = max_index(probs_[i], num_classes);
    float prob = probs_[i][class_id];
    if (prob > 0.0) {
      tmc_yolo2_ros::Detection detection;
      box b = boxes_[i];
      detection.x = b.x * image_w;
      detection.y = b.y * image_h;
      detection.width = b.w * image_w;
      detection.height = b.h * image_h;
      detection.confidence = prob;
      detection.class_id = class_id;
      detection.class_name = class_names_[class_id];
      detections.push_back(detection);
    }
  }
  return detections;
}
}  // namespace tmc_yolo2_ros
