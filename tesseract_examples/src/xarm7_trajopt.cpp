/**
 * @file xarm7_trajop_upright_example.cpp
 * @brief xarm7_trajop  example implementation
 *
 * @author Levi Armstrong
 * @date July 22, 2019
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <json/json.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_examples/xarm7_trajopt.h>
#include <tesseract_environment/utils.h>
#include <tesseract_common/timer.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>
#include <tesseract_task_composer/core/task_composer_input.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>

#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#include <tesseract_geometry/impl/sphere.h>

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_planning;
using tesseract_common::ManipulatorInfo;

static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";

namespace tesseract_examples
{
Xarm7Trajopt::Xarm7Trajopt(tesseract_environment::Environment::Ptr env,
                          tesseract_visualization::Visualization::Ptr plotter
                          ) : env_(std::move(env)), plotter_(std::move(plotter))
{
}

// run with arm joint angle start and cartesion position end
bool Xarm7Trajopt::run(Eigen::VectorXd joint_start_pos, Eigen::Isometry3d final_pose)
{

  if (plotter_ != nullptr)
    plotter_->waitForConnection();

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.emplace_back("joint1");
  joint_names.emplace_back("joint2");
  joint_names.emplace_back("joint3");
  joint_names.emplace_back("joint4");
  joint_names.emplace_back("joint5");
  joint_names.emplace_back("joint6");
  joint_names.emplace_back("joint7");

  env_->setState(joint_names, joint_start_pos);

  tesseract_common::VectorIsometry3d transforms;
  transforms = env_->getLinkTransforms();

  Eigen::Matrix3d m;
  Eigen::Vector3d v;

  for(int i=0; i < transforms.size(); i++){
    std::cout << "transform: " << std::endl << i << std::endl;
    m = transforms[i].rotation();
    v = transforms[i].translation();
    std::cout << "Rotation: " << std::endl << m << std::endl;
    std::cout << "Translation: " << std::endl << v << std::endl;
    Eigen::Quaterniond quat(m.cast<double>());

    std::cout << "Debug: " << "myQuaternion.w() = " << quat.w() << std::endl; //Print out the scalar
    std::cout << "Debug: " << "myQuaternion.vec() = " << quat.vec() << std::endl; //Print out the orientation vector
  }

  Eigen::Quaterniond selected_quat(transforms[10].rotation().cast<double>());
  std::cout << "selected_quat: " << "myQuaternion.w() = " << selected_quat.w() << std::endl; //Print out the scalar
  std::cout << "selected_quat: " << "myQuaternion.vec() = " << selected_quat.vec() << std::endl; //Print out the orientation vector


  
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // Solve Trajectory
  CONSOLE_BRIDGE_logInform("xarm7_trajop upright plan example");

  // Create Task Composer Plugin Factory
  const std::string share_dir(TESSERACT_TASK_COMPOSER_DIR);
  tesseract_common::fs::path config_path(share_dir + "/config/task_composer_plugins.yaml");
  TaskComposerPluginFactory factory(config_path);

  // Create Program
  CompositeInstruction program(
      "FREESPACE", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator", "base_link", "tool0"));

  // Start and End Joint Position for the program
  StateWaypointPoly wp0{ StateWaypoint(joint_names, joint_start_pos) };
  
  

  // Define the final pose
  // final_pose_.linear() = selected_quat.matrix();
  std::cout << "final_pose_ rotation "<< final_pose_.rotation() << std::endl; //Print out the scalar
  std::cout << "final_pose_ translation "<< final_pose_.translation() << std::endl; //Print out the scalar
  CartesianWaypointPoly pick_wp1{ CartesianWaypoint(final_pose_) };

  MoveInstruction start_instruction(wp0, MoveInstructionType::FREESPACE, "FREESPACE");
  start_instruction.setDescription("Start Instruction");

  // Plan freespace from start
  // Assign a linear motion so cartesian is defined as the target
  MoveInstruction plan_f0(pick_wp1, MoveInstructionType::FREESPACE, "FREESPACE");
  plan_f0.setDescription("freespace_plan");

  // Add Instructions to program
  program.appendMoveInstruction(start_instruction);
  program.appendMoveInstruction(plan_f0);

  // Print Diagnostics
  program.print("Program: ");

  // Create Executor
  auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");

  // Create profile dictionary
  auto profiles = std::make_shared<ProfileDictionary>();
  
  auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  composite_profile->collision_cost_config.enabled = true;
  composite_profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  composite_profile->collision_cost_config.safety_margin = 0.01;
  composite_profile->collision_cost_config.safety_margin_buffer = 0.01;
  composite_profile->collision_cost_config.coeff = 1;
  composite_profile->collision_constraint_config.enabled = true;
  composite_profile->collision_constraint_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  composite_profile->collision_constraint_config.safety_margin = 0.01;
  composite_profile->collision_constraint_config.safety_margin_buffer = 0.01;
  composite_profile->collision_constraint_config.coeff = 1;
  composite_profile->smooth_velocities = true;
  composite_profile->smooth_accelerations = false;
  composite_profile->smooth_jerks = false;
  composite_profile->velocity_coeff = Eigen::VectorXd::Ones(1);
  profiles->addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE", composite_profile);

  auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 5);
  // plan_profile->cartesian_coeff(0) = 0;
  // plan_profile->cartesian_coeff(1) = 0;
  // plan_profile->cartesian_coeff(2) = 0;

  // Add profile to Dictionary
  profiles->addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE", plan_profile);

  auto trajopt_solver_profile = std::make_shared<TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->opt_info.max_iter = 100;
  profiles->addProfile<TrajOptSolverProfile>(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE", trajopt_solver_profile);


  // Create task
  const std::string task_name = "TrajOptPipeline";
  TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
  const std::string input_key = task->getInputKeys().front();
  const std::string output_key = task->getOutputKeys().front();

  // Create Task Input Data
  TaskComposerDataStorage input_data;
  input_data.setData(input_key, program);

  // Create Task Composer Problem
  auto problem = std::make_unique<PlanningTaskComposerProblem>(env_, input_data, profiles);

  if (plotter_ != nullptr && plotter_->isConnected())
    plotter_->waitForInput("Hit Enter to solve for trajectory.");

  // Solve process plan
  tesseract_common::Timer stopwatch;
  stopwatch.start();
  TaskComposerInput input(std::move(problem));
  TaskComposerFuture::UPtr future = executor->run(*task, input);
  future->wait();

  stopwatch.stop();
  CONSOLE_BRIDGE_logInform("Planning took %f seconds.", stopwatch.elapsedSeconds());

  // Plot Process Trajectory
  if (plotter_ != nullptr && plotter_->isConnected())
  {
    plotter_->waitForInput();
    auto ci = input.data_storage.getData(output_key).as<CompositeInstruction>();
    tesseract_common::Toolpath toolpath = toToolpath(ci, *env_);
    tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
    auto state_solver = env_->getStateSolver();

    plotter_->plotMarker(ToolpathMarker(toolpath));
    plotter_->plotTrajectory(trajectory, *state_solver);

    // print trajecotry length:
    std::cout << "trajectory length: " << trajectory.size() << std::endl;

    // loop through joint_trajecotry and print
    for (const auto& js : trajectory)
    {
      std::cout << "joint state " << std::endl;
      assert(js.joint_names.size() == static_cast<unsigned>(js.position.size()));

      for (int i = 0; i < js.position.size(); ++i)
        std::cout << "pos"  <<  js.position(i) << std::endl;

      for (int i = 0; i < js.velocity.size(); ++i)
        std::cout << "vell"  << js.velocity(i) << std::endl;

      for (int i = 0; i < js.acceleration.size(); ++i)
        std::cout << "accel"  <<  js.acceleration(i) << std::endl;

      std::cout << "time"  <<  js.time << std::endl;
    }


  }

  // CONSOLE_BRIDGE_logInform("Final trajectory is collision free");
  std::cout << "success: " << input.isSuccessful() << std::endl;
  return input.isSuccessful();
}

}
