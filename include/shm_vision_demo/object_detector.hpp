// Copyright 2021 Matthias Killiat
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
#ifndef SHM_VISION_DEMO__OBJECT_DETECTOR_HPP_
#define SHM_VISION_DEMO__OBJECT_DETECTOR_HPP_
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <string>
#include <vector>

#include <fstream>


class ObjectDetector
{
public:
  ObjectDetector(const std::string & configFolder, int width, int height)
  : netInputWidth(width), netInputHeight(height)
  {
    classesFile = configFolder + classesFile;
    modelFile = configFolder + modelFile;
    weightsFile = configFolder + weightsFile;
    std::cout << "using dnn weights from " << weightsFile << std::endl;

    init();
  }

  void process_frame(cv::Mat & frame)
  {
    cv::Size size(netInputWidth, netInputHeight);
    cv::dnn::blobFromImage(
      frame, m_blob, 1 / 255.0, size, cv::Scalar(0, 0, 0),
      true, false);

    m_net.setInput(m_blob);
    std::vector<cv::Mat> outs;
    m_net.forward(outs, getOutputNames(m_net));

    postprocess(frame, outs);

    frame.convertTo(m_result, CV_8U);
  }

  cv::Mat & get_result() {return m_result;}

private:
  std::string classesFile{"coco.names"};
  std::string modelFile{"yolov3.cfg"};
  std::string weightsFile{"yolov3.weights"};

  double confThreshold{0.5};
  double nmsThreshold{0.4};
  int netInputWidth;
  int netInputHeight;

  cv::dnn::Net m_net;
  cv::Mat m_blob;
  cv::Mat m_result;
  std::vector<std::string> classes;

  void init()
  {
    cv::String modelConfiguration{modelFile.c_str()};
    cv::String modelWeights{weightsFile.c_str()};

    m_net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
    // m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    // m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    // m_net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);

    m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    std::ifstream f(classesFile.c_str());
    std::string line;
    while (std::getline(f, line)) {
      classes.push_back(line);
    }
  }

  std::vector<cv::String> getOutputNames(const cv::dnn::Net & net)
  {
    static std::vector<cv::String> names;
    if (names.empty()) {
      std::vector<int> outLayers = net.getUnconnectedOutLayers();
      std::vector<cv::String> layerNames = net.getLayerNames();
      names.resize(outLayers.size());
      for (size_t i = 0; i < outLayers.size(); ++i) {
        names[i] = layerNames[outLayers[i] - 1];
      }
    }
    return names;
  }

  void postprocess(cv::Mat & frame, const std::vector<cv::Mat> & outs)
  {
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i) {
      float * data = reinterpret_cast<float *>(outs[i].data);
      for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
        cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
        cv::Point classIdPoint;
        double confidence;

        cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
        if (confidence > confThreshold) {
          int centerX = static_cast<int>(data[0] * frame.cols);
          int centerY = static_cast<int>(data[1] * frame.rows);
          int width = static_cast<int>(data[2] * frame.cols);
          int height = static_cast<int>(data[3] * frame.rows);
          int left = centerX - width / 2;
          int top = centerY - height / 2;

          classIds.push_back(classIdPoint.x);
          confidences.push_back(static_cast<float>(confidence));
          boxes.push_back(cv::Rect(left, top, width, height));
        }
      }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i) {
      int idx = indices[i];
      cv::Rect box = boxes[idx];
      drawBoxes(
        classIds[idx], confidences[idx], box.x, box.y,
        box.x + box.width, box.y + box.height, frame);
    }
  }

  void drawBoxes(
    int classId, float conf, int left, int top, int right,
    int bottom, cv::Mat & frame)
  {
    cv::rectangle(
      frame, cv::Point(left, top), cv::Point(right, bottom),
      cv::Scalar(255, 178, 50), 3);

    std::string label = cv::format("%.2f", conf);
    if (!classes.empty()) {
      CV_Assert(classId < static_cast<int>(classes.size()));
      label = classes[classId] + ":" + label;
    }

    int baseLine;
    cv::Size labelSize =
      cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = std::max(top, labelSize.height);
    cv::rectangle(
      frame, cv::Point(left, top - std::round(1.5 * labelSize.height)),
      cv::Point(left + std::round(1.5 * labelSize.width), top + baseLine),
      cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(
      frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX,
      0.75, cv::Scalar(0, 0, 0), 1);
  }
};
#endif  // SHM_VISION_DEMO__OBJECT_DETECTOR_HPP_
