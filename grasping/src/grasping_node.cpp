#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "grasping_interface/srv/two_points.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "thread"
#include "chrono"
#include "franka/gripper.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("grasping");
namespace mtc = moveit::task_constructor;

using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask(geometry_msgs::msg::PoseStamped target, geometry_msgs::msg::PoseStamped goal);

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(geometry_msgs::msg::PoseStamped target, geometry_msgs::msg::PoseStamped goal);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  //Put objects for your planningscene here (collision objects etc)
}

void MTCTaskNode::doTask(geometry_msgs::msg::PoseStamped target, geometry_msgs::msg::PoseStamped goal)
{
  int state = 0;
  std::cout <<  "--------------------- 1 -------------------" << std::endl;
  task_ = createTask(target, goal);
  std::cout <<  "--------------------- 2 -------------------" << std::endl;


  try
  {
    task_.init();
    std::cout <<  "--------------------- 3 -------------------" << std::endl;

  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    std::cout <<  "--------------------- 4 -------------------" << std::endl;

    return;
  }
  std::cout <<"DoTask 1"<< std::endl;
  if (!task_.plan(5 /* max_solutions */))
  {

    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());
  
  std::cout <<"DoTask 2"<< std::endl;

  //auto joint_trajectory = *task_.solutions().front()->trajectory();
  //std::cout << "Type of num: " << *task_.solutions().front().toMsg() << std::endl;
  moveit_task_constructor_msgs::action::ExecuteTaskSolution::Goal goal_pass;
  std::cout <<  "--------------------- 5 -------------------" << std::endl;

  auto result = task_.execute2(*task_.solutions().front(), goal_pass);
  
  //call actionserver----------------------------------------------------------
  


	// cooool code 
	auto solution = goal_pass.solution;
	//std::cout << typeid(solution).name() << std::endl;

	//std::cout << "Task_Id: " << solution.task_id << std::endl;
	std::cout << "Len Arr: " << solution.sub_trajectory.size() << std::endl;
  
  auto gripper = franka::Gripper("192.168.1.200");
  //solution.sub_trajectory.size()-1
  for (unsigned int i=1;i<solution.sub_trajectory.size();i++){
    
    auto subtrajectory = solution.sub_trajectory[i];
    std::cout << typeid(subtrajectory).name() << std::endl;
    auto traj = subtrajectory.trajectory.joint_trajectory;
    std::cout << typeid(traj).name() << std::endl;
    std::cout << traj.joint_names.size() << std::endl;
    
    
    if (traj.joint_names.size()!=1){
      // build goal object
      FollowJTrajAction::Goal goal_traj;
      goal_traj.trajectory = traj;

      std::cout << "Len Traj:  " << traj.points.size() << std::endl;
      //auto traj_point = traj.points[0]

      std::cout << traj.joint_names.size() << std::endl;
      // create client
      auto ac2 = rclcpp_action::create_client<FollowJTrajAction>(
          node_, "/joint_trajectory_controller/follow_joint_trajectory");
        std::cout << "Huuuuuuge Problem !!" << std::endl;
        ac2->wait_for_action_server();
      std::cout << "Huge Problem !!!" << std::endl;
      // send goal
      auto goal_handle_future2 = ac2->async_send_goal(goal_traj);
      if (rclcpp::spin_until_future_complete(node_, goal_handle_future2) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Send goal call failed");
        std::cout << "Huuuuuuge Problem !!!" << std::endl;
      }
    }
    else if(i == 1 || i == 5){
      std::cout << "grasp Gripper open" << std::endl;
      double width = 0.07;
      double speed = width;
      double epsilon_inner = 0.00;
      double epsilon_outer = 0.07;
      gripper.grasp(width, speed, 5,epsilon_inner, epsilon_outer);
    }
    else {
      std::cout << "grasp Gripper close" << std::endl;
      double width = 0.001;
      double speed = 0.07;
      double epsilon_inner = 0.00;
      double epsilon_outer = 0.07;
      gripper.grasp(width, speed, 5,epsilon_inner, epsilon_outer);
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }
  //call actionserver end---------------------------------------------------
  
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    //return;    change back !!!!!!!!!!!!!!!!!!!!
  }
  std::cout <<"DoTask END"<< std::endl;
  return;
}

mtc::Task MTCTaskNode::createTask(geometry_msgs::msg::PoseStamped target, geometry_msgs::msg::PoseStamped goal)
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  
  // Stage initializer, TODO: Track the stages

  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

 // Loads cartesian, sampling and interpolation planner
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  //  All stages are defined here. For future iterations think about using a Serial Container

  // Open Hand
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // Move to Object
  auto stage_move = std::make_unique<mtc::stages::MoveTo>("move to grasp", sampling_planner);
  stage_move->setGroup(arm_group_name);
 
  stage_move->setGoal(target);
  task.add(std::move(stage_move));

  //Close Hand around object
  auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
  stage_close_hand->setGroup(hand_group_name);
  stage_close_hand->setGoal("close");
  task.add(std::move(stage_close_hand));

  //Move to Place Position
  auto stage_move_2 = std::make_unique<mtc::stages::MoveTo>("move to place", sampling_planner);
  stage_move_2->setGroup(arm_group_name);

  stage_move_2->setGoal(goal);
  task.add(std::move(stage_move_2));

  // Open Hand
  auto stage_open_hand_2 = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
  stage_open_hand_2->setGroup(hand_group_name);
  stage_open_hand_2->setGoal("open");
  task.add(std::move(stage_open_hand_2));

  // Close Hand
  auto stage_neutral = std::make_unique<mtc::stages::MoveTo>("ready position", sampling_planner);
  stage_neutral->setGroup(arm_group_name);
  stage_neutral->setGoal("ready");
  task.add(std::move(stage_neutral));

  return task;
}

//Service function
void pickplace(const std::shared_ptr<grasping_interface::srv::TwoPoints::Request> request,
          std::shared_ptr<grasping_interface::srv::TwoPoints::Response> response){
  
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin_once();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask(request->target,request->goal);
  spin_thread->join();
  std::cout <<"Thread Join Done"<< std::endl;
  response->state = 0;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("TwoPoints_Server");

  rclcpp::Service<grasping_interface::srv::TwoPoints>::SharedPtr service = 
    node->create_service<grasping_interface::srv::TwoPoints>("Pick_Place", &pickplace);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to move.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}