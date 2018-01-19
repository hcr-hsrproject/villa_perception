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
#ifndef TMC_YOLO2_DETECTOR_HPP
#define TMC_YOLO2_DETECTOR_HPP
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tmc_yolo2_ros/Detection.h>

extern "C" {
#undef __cplusplus
#include <tmc_darknet/box.h>
#include <tmc_darknet/image.h>
#include <tmc_darknet/network.h>
#define __cplusplus
}

namespace tmc_yolo2_ros {
class Detector {
 public:
  Detector() {}
  virtual ~Detector() {
    free(probs_[0]);
    free_network(net_);
  }
  /// @brief 認識器の初期化を行う
  /// @param[in] config_path ネットワーク定義ファイル(.cfg)のパス
  /// @param[in] weights_path 学習結果の重みファイル(.weights)のパス
  /// @param[in] class_names_path 認識するクラス名リストファイル(.names)のパス
  /// @param[in] confidence_threshold 認識結果として採用する信頼度の閾値
  /// @param[in] nms Non-Maximum suppression処理を行う際の重複領域判定の閾値
  /// @param[out] dst_width ネットワークにおける入力画像の幅
  /// @param[out] dst_height ネットワークにおける入力画像の高さ
  /// @return 初期化に成功したか
  virtual bool Initialize(const std::string& config_path,
                          const std::string& weights_path,
                          const std::string& class_names_path,
                          double confidence_threshold,
                          double nms_threshold,
                          uint32_t& dst_width,
                          uint32_t& dst_height);

  /// @brief 認識処理を実行する
  /// @param[in] darknet_image 入力画像
  /// @param[in] image_w 入力画像の(ネットワークに合わせてリサイズする前の)幅
  /// @param[in] image_h 入力画像の(ネットワークに合わせてリサイズする前の)高さ
  /// @return 認識結果
  virtual std::vector<tmc_yolo2_ros::Detection> Execute(image* darknet_image,
                                                        uint32_t image_w,
                                                        uint32_t image_h);

 private:
  /// 認識結果として採用する信頼度の閾値 [0.0, 1.0]
  double confidence_threshold_;
  /// Non-Maximum suppression処理を行う際の重複領域判定(IoU)の閾値 [0.0, 1.0]
  double nms_threshold_;
  /// 認識器のネットワーク
  network net_;
  /// 認識領域
  std::vector<box> boxes_;
  /// 認識領域の尤度
  std::vector<float *> probs_;
  /// クラス名リスト
  std::vector<std::string> class_names_;
};
}  // namespace tmc_yolo2_ros

#endif  // TMC_YOLO2_DETECTOR_HPP
