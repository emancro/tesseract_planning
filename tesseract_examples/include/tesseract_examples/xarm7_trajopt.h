/**
 * @file glass_upright_example.h
 * @brief An example of a robot with fixed orientation but free to move in cartesian space.
 *
 * @author Levi Armstrong
 * @date July 22, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_EXAMPLES_GLASS_UPRIGHT_EXAMPLE_H
#define TESSERACT_EXAMPLES_GLASS_UPRIGHT_EXAMPLE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/example.h>

namespace tesseract_examples
{
/**
 * @brief An example of a robot with fixed orientation but free to move in cartesian space
 * leveraging tesseract and trajopt to generate a motion trajectory.
 */
class Xarm7Trajopt : public Example
{
public:
  Xarm7Trajopt(tesseract_environment::Environment::Ptr env,
                      tesseract_visualization::Visualization::Ptr plotter = nullptr,
                      bool ifopt = false,
                      bool debug = false,
                      Eigen::Vector3d sphere1 = Eigen::Vector3d(0, 0, 0),
                      Eigen::VectorXd arm_start = Eigen::VectorXd::Zero(7),
                      Eigen::VectorXd arm_end = Eigen::VectorXd::Zero(7)
                      );
  ~Xarm7Trajopt() override = default;
  Xarm7Trajopt(const Xarm7Trajopt&) = default;
  Xarm7Trajopt& operator=(const Xarm7Trajopt&) = default;
  Xarm7Trajopt(Xarm7Trajopt&&) = default;
  Xarm7Trajopt& operator=(Xarm7Trajopt&&) = default;

  bool run() override final;

private:
  bool ifopt_;
  bool debug_;
  Eigen::Vector3d sphere1_;
  Eigen::VectorXd arm_start_;
  Eigen::VectorXd arm_end_;
  static tesseract_environment::Command::Ptr addSphere(Eigen::Vector3d sphere1);
};

}  // namespace tesseract_examples

#endif  // TESSERACT_EXAMPLES_GLASS_UPRIGHT_EXAMPLE_H
