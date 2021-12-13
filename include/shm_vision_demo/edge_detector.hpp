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
#ifndef SHM_VISION_DEMO__EDGE_DETECTOR_HPP_
#define SHM_VISION_DEMO__EDGE_DETECTOR_HPP_
#include <opencv2/opencv.hpp>

#include "filter.hpp"

// TODO(matthiaskillat): add SIFT features

namespace demo
{

class EdgeDetector
{
public:
  void sobel(const cv::Mat & in, cv::Mat & out)
  {
    cv::Sobel(in, out, CV_8U, 1, 1, 5);
  }

  void laplace(const cv::Mat & in, cv::Mat & out)
  {
    cv::Laplacian(in, out, in.type(), 3, 1, 0, cv::BORDER_DEFAULT);
  }

  void canny(const cv::Mat & in, cv::Mat & out, int t1 = 100, int t2 = 150)
  {
    cv::Canny(in, out, t1, t2, 3);
  }

private:
};

}  // namespace demo
#endif  // SHM_VISION_DEMO__EDGE_DETECTOR_HPP_
