// Copyright 2016 Open Source Robotics Foundation, Inc.
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

// %Tag(FULLTEXT)%
// %Tag(INCLUDE_STATEMENTS)%
#include <algorithm>
#include <vector>
#include <string>
#include <deque>
#include <queue>


#include <ros/ros.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <std_srvs/Trigger.h>
//#include <std_srvs/SetBool.h>

#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/ConveyorBeltState.h>
#include <osrf_gear/VacuumGripperState.h>
#include <osrf_gear/AGVControl.h>

#include <trajectory_msgs/JointTrajectory.h>
// %EndTag(INCLUDE_STATEMENTS)%

// %Tag(START_COMP)%
/// Start the competition by waiting for and then calling the start ROS Service.

#define MAIN_FILE
using namespace std;

vector<double> kinematic(vector<double> theta);
vector<double> invkinematic(vector<double> pose);

enum State{
  IDLE,
  TRANSFER,
  TRANSIT
};

void start_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}
// %EndTag(START_COMP)%

/// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle & node)
  : current_score_(0), arm_1_has_been_zeroed_(false), arm_2_has_been_zeroed_(false)
  {
    // %Tag(ADV_CMD)%
    arm_1_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm1/arm/command", 10);

    arm_2_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm2/arm/command", 10);
    // %EndTag(ADV_CMD)%

    arm_1_gripper_ctrl = node.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm1/gripper/control");
    arm_2_gripper_ctrl = node.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm2/gripper/control");

    agv_1 = node.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/agv1");
    agv_2 = node.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/agv2");

    arm_1_joint.resize(6);
    arm_2_joint.resize(6);

    arm_1_joint_goal.resize(6);
    arm_2_joint_goal.resize(6);

    //idle = true;
    arm_1_state = IDLE;
    arm_2_state = IDLE;

    catched_1 = false;
    catched_2 = false;
  }



  /// Called when a new message is received.
  void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
    if (msg->data != current_score_)
    {
      ROS_INFO_STREAM("Score: " << msg->data);
    }
    current_score_ = msg->data;
  }


  /// Called when a new message is received.
  void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
    if (msg->data == "done" && competition_state_ != "done")
    {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }


  /// Called when a new Order message is received.
  void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
  }


  //stage:
  //idle
  //transit
  //transfer

  // 50 HZ
  /// Called when a new JointState message is received.
  void arm_1_joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    arm_1_current_joint_states_ = *joint_state_msg;
    
    arm_1_joint[0] = arm_1_current_joint_states_.position[3]; //shoulder pan
    arm_1_joint[1] = arm_1_current_joint_states_.position[2]; //shoulder lift
    arm_1_joint[2] = arm_1_current_joint_states_.position[0]; //elbow
    arm_1_joint[3] = arm_1_current_joint_states_.position[4]; //wrist1
    arm_1_joint[4] = arm_1_current_joint_states_.position[5]; //wrist2
    arm_1_joint[5] = arm_1_current_joint_states_.position[6]; //wrist3
    arm_1_linear = arm_1_current_joint_states_.position[1]; //linear
    
    switch(arm_1_state){
      case IDLE:
        if(!event.empty()){
          ros::Duration tmp = ros::Time::now() - event[0];

          double ttc = 1.5;
          double dist = (tmp.toSec() + ttc) * belt_power/100 * maxBeltVel + 0.92 - 2.25 - 0.02;
          //double linear = 0;
          //if(fabs(dist)>0.1)
          send_arm_to_state_2(arm_1_joint_trajectory_publisher_, 
            invkinematic(vector<double>{-0.92, -belt_power / 100 * maxBeltVel * ttc / 4, 0.03}), 
            invkinematic(vector<double>{-0.92, 0, -0.07}), ttc, -dist);
          event.pop_front();
          arm_1_state = TRANSIT;
        }
        break;
      case TRANSIT:
        open_gripper(1);
        if(catched_1){
          arm_1_state=TRANSFER;
          send_arm_to_state_2(arm_1_joint_trajectory_publisher_, invkinematic(vector<double>{0.0,-0.9,0.0}), 
            invkinematic(vector<double>{0.0, -0.9, -0.1}), 1.5, 2.35);
        }
        break;
      case TRANSFER:
        if(reached(arm_1_joint, arm_1_joint_goal)){
          close_gripper(1);
          arm_1_state = IDLE;
        }
        break;
    }

    if (!arm_1_has_been_zeroed_) {
      arm_1_has_been_zeroed_ = true;
      ROS_INFO("Sending arm to zero joint positions...");
      send_arm_to_zero_state(arm_1_joint_trajectory_publisher_);
    }
  }

  bool reached(vector<double>&s, vector<double>& t){
    double sum=0;
    for(int i=0;i<6;i++){
      sum += min(fabs(s[i]-t[i]), fabs(6.28318530718 - fabs(s[i]-t[i])));
    }
    //cout<<sum<<endl;
    return sum < 3e-2;
  }

  void arm_2_joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Joint States arm 2 (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    arm_2_current_joint_states_ = *joint_state_msg;

    arm_2_joint[0] = arm_2_current_joint_states_.position[3]; //shoulder pan
    arm_2_joint[1] = arm_2_current_joint_states_.position[2]; //shoulder lift
    arm_2_joint[2] = arm_2_current_joint_states_.position[0]; //elbow
    arm_2_joint[3] = arm_2_current_joint_states_.position[4]; //wrist1
    arm_2_joint[4] = arm_2_current_joint_states_.position[5]; //wrist2
    arm_2_joint[5] = arm_2_current_joint_states_.position[6]; //wrist3
    arm_2_linear = arm_2_current_joint_states_.position[1]; //linear
    
    if (!arm_2_has_been_zeroed_) {
      arm_2_has_been_zeroed_ = true;
      ROS_INFO("Sending arm 2 to zero joint positions...");
      

      //send_arm_to_zero_state(arm_2_joint_trajectory_publisher_);


    }
  }
  // %EndTag(CB_CLASS)%

  // %Tag(ARM_ZERO)%
  /// Create a JointTrajectory with all positions set to zero, and command the arm.
  void send_arm_to_zero_state(ros::Publisher & joint_trajectory_publisher) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;

    // Fill the names of the joints to be controlled.
    // Note that the vacuum_gripper_joint is not controllable.
    msg.joint_names.clear();
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    msg.joint_names.push_back("linear_arm_actuator_joint");
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);

    // -0.92 -0.2 0
    auto res = invkinematic(vector<double>{0,-0.9, 0.15});
    msg.points[0].positions[0]=res[0];
    msg.points[0].positions[1]=res[1];
    msg.points[0].positions[2]=res[2];
    msg.points[0].positions[3]=res[3];
    msg.points[0].positions[4]=res[4];
    msg.points[0].positions[5]=res[5];
    msg.points[0].positions[6]=2.35;

    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(0.5);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
  }



  void send_arm_to_state(ros::Publisher & joint_trajectory_publisher, std::vector<double> joints, double t, double linear=0.0) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;

    if(joint_trajectory_publisher == arm_1_joint_trajectory_publisher_)
      arm_1_joint_goal = joints;
    else
      arm_2_joint_goal = joints;
    // Fill the names of the joints to be controlled.
    // Note that the vacuum_gripper_joint is not controllable.
    msg.joint_names.clear();
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    msg.joint_names.push_back("linear_arm_actuator_joint");
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    for(int i=0;i<6;i++){
      msg.points[0].positions[i] = joints[i];
    }
    msg.points[0].positions[6] = linear;
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(t);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
  }  

  void send_arm_to_state_2(ros::Publisher & joint_trajectory_publisher, std::vector<double> joints1, std::vector<double> joints2, double t, double linear=0.0) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;

    if(joint_trajectory_publisher == arm_1_joint_trajectory_publisher_)
      arm_1_joint_goal = joints2;
    else
      arm_2_joint_goal = joints2;
    // Fill the names of the joints to be controlled.
    // Note that the vacuum_gripper_joint is not controllable.
    msg.joint_names.clear();
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    msg.joint_names.push_back("linear_arm_actuator_joint");
    // Create one point in the trajectory.
    msg.points.resize(2);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    for(int i=0;i<6;i++){
      msg.points[0].positions[i] = joints1[i];
    }
    msg.points[0].positions[6] = linear;
    
    msg.points[1].positions.resize(msg.joint_names.size(), 0.0);
    for(int i=0;i<6;i++){
      msg.points[1].positions[i] = joints2[i];
    }
    msg.points[1].positions[6] = linear;

    cout<<"got here"<<endl;
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(ros::Duration(t).toSec()*3/4);
    msg.points[1].time_from_start = ros::Duration(t);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
  }

  // %EndTag(ARM_ZERO)%

  /// Called when a new LogicalCameraImage message is received.
  void logical_camera_callback(
    const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Logical camera: '" << image_msg->models.size() << "' objects.");
    // int i = 0;
    // for(auto &item: image_msg->models){
    //   ROS_INFO_STREAM_THROTTLE(10+i,
    //   " item: " << ++i <<": " << item << "----");
    // }
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "\n");
  }

  /// Called when a new Proximity message is received.
  void break_beam_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_INFO("Break beam triggered.");
      event.emplace_back(ros::Time::now());
      // if(idle)
      //   {
      //     send_arm_to_state(arm_1_joint_trajectory_publisher_, invkinematic(vector<double>{-0.92, 0.0, -0.07}), 6.5);
      //     idle=false;
      //   }
    }
  }
  // bool idle;

  void to_agv1(double x, double y, double theta){

  }

  void to_agv2(double x, double y, double theta){
    
  }
  
  void belt_state_callback(const osrf_gear::ConveyorBeltState::ConstPtr & msg){
    belt_power = msg->power;
  }

  void open_gripper(int num){
    osrf_gear::VacuumGripperControl srv;
    srv.request.enable = true;
    if(num==1){
      arm_1_gripper_ctrl.call(srv);
    }
    else{
      arm_2_gripper_ctrl.call(srv);
    }

    if(!srv.response.success){
      ROS_ERROR_STREAM("Gripper Failed");
    }
    else{
      ROS_INFO("Gripper openned");
    }
  }

  void agv(int num, int order, int kit){
    osrf_gear::AGVControl srv;
    srv.request.shipment_type = string("order_") + to_string(order) + string("kit_") + to_string(kit);
    if(num==1){
      agv_1.call(srv);
    }
    else{
      agv_2.call(srv);
    }
  }

  void gripper_1_callback(const osrf_gear::VacuumGripperState::ConstPtr & msg){
    catched_1 = msg->attached;
    // if(catched1){
    //   if(!msg->attached){
    //     transfer1=false;
    //   }
    // }
    // //transit
    // else{    
    //   if(msg->attached){
    //     // catched1=true;
        
    //     // auto pos = kinematic(arm_1_joint);
    //     // pos[2] += 0.1 ;
    //     // auto joint = invkinematic(pos);
    //     // send_arm_to_state(arm_1_joint_trajectory_publisher_, joint, 0.2);

    //     //cout<<"catched"<<endl;  /**/
    //   }
    // }
  }

  void gripper_2_callback(const osrf_gear::VacuumGripperState::ConstPtr & msg){
    catched_2 = msg->attached;
    // if(!transfer1){
    //   if(!msg->attached){
    //     transfer1=false;
    //   }
    // }
    // else{
    //   if(msg->attached){
    //     transfer2=true;
    //   }
    // }
  }

  void close_gripper(int num){
    osrf_gear::VacuumGripperControl srv;
    srv.request.enable = false;
    if(num==1){
      arm_1_gripper_ctrl.call(srv);
    }
    else{
      arm_2_gripper_ctrl.call(srv);
    }

    if(!srv.response.success){
      ROS_ERROR_STREAM("Gripper Failed");
    }
    else{
      ROS_INFO("Gripper closed");
    }

  }

private:
  std::string competition_state_;
  double current_score_;

  double belt_power;

  bool transfer1, transfer2;

  vector<double> arm_1_joint;
  vector<double> arm_2_joint;

  vector<double> arm_1_joint_goal, arm_2_joint_goal;

  double arm_1_linear, arm_2_linear;

  deque<ros::Time> event;

  const double maxBeltVel = 0.2;

  ros::Publisher arm_1_joint_trajectory_publisher_;
  ros::Publisher arm_2_joint_trajectory_publisher_;
  
  ros::ServiceClient arm_1_gripper_ctrl;
  ros::ServiceClient arm_2_gripper_ctrl;
  ros::ServiceClient agv_1;
  ros::ServiceClient agv_2;

  std::vector<osrf_gear::Order> received_orders_;
  sensor_msgs::JointState arm_1_current_joint_states_;
  sensor_msgs::JointState arm_2_current_joint_states_;
  bool arm_1_has_been_zeroed_;
  bool arm_2_has_been_zeroed_;

  State arm_1_state;
  State arm_2_state;

  bool catched_1, catched_2;

};

void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
  if ((msg->max_range - msg->range) > 0.01) {  // If there is an object in proximity.
    ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
  }
}

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
  size_t number_of_valid_ranges = std::count_if(
    msg->ranges.begin(), msg->ranges.end(), [](const float f) {return std::isfinite(f);});
  if (number_of_valid_ranges > 0) {
    ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
  }
}

// %Tag(MAIN)%
int main(int argc, char ** argv) {
  // Last argument is the default name of the node.
  ros::init(argc, argv, "ariac_example_node");

  ros::NodeHandle node;

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);

  // Subscribe to the '/ariac/current_score' topic.
  ros::Subscriber current_score_subscriber = node.subscribe(
    "/ariac/current_score", 10,
    &MyCompetitionClass::current_score_callback, &comp_class);

  // Subscribe to the '/ariac/competition_state' topic.
  ros::Subscriber competition_state_subscriber = node.subscribe(
    "/ariac/competition_state", 10,
    &MyCompetitionClass::competition_state_callback, &comp_class);

  // %Tag(SUB_CLASS)%
  // Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = node.subscribe(
    "/ariac/orders", 10,
    &MyCompetitionClass::order_callback, &comp_class);
  // %EndTag(SUB_CLASS)%

  // Subscribe to the '/ariac/joint_states' topic.
  ros::Subscriber arm_1_joint_state_subscriber = node.subscribe(
    "/ariac/arm1/joint_states", 10,
    &MyCompetitionClass::arm_1_joint_state_callback, &comp_class);

  ros::Subscriber arm_2_joint_state_subscriber = node.subscribe(
    "/ariac/arm2/joint_states", 10,
    &MyCompetitionClass::arm_2_joint_state_callback, &comp_class);

  // %Tag(SUB_FUNC)%
  // Subscribe to the '/ariac/proximity_sensor_1' topic.
  ros::Subscriber proximity_sensor_subscriber = node.subscribe(
    "/ariac/proximity_sensor_1", 10, proximity_sensor_callback);
  // %EndTag(SUB_FUNC)%

  // Subscribe to the '/ariac/break_beam_1_change' topic.
  ros::Subscriber break_beam_subscriber = node.subscribe(
    "/ariac/break_beam_1_change", 10,
    &MyCompetitionClass::break_beam_callback, &comp_class);

  // Subscribe to the '/ariac/logical_camera_1' topic.
  ros::Subscriber logical_camera_subscriber = node.subscribe(
    "/ariac/logical_camera_1", 10,
    &MyCompetitionClass::logical_camera_callback, &comp_class);

  // Subscribe to the '/ariac/laser_profiler_1' topic.
  ros::Subscriber laser_profiler_subscriber = node.subscribe(
    "/ariac/laser_profiler_1", 10, laser_profiler_callback);

  ros::Subscriber belt_state_subscriber = node.subscribe(
    "/ariac/conveyor/state", 10, 
    &MyCompetitionClass::belt_state_callback, &comp_class);
  
  ros::Subscriber gripper_1_state_subscriber = node.subscribe(
    "/ariac/arm1/gripper/state", 10, 
    &MyCompetitionClass::gripper_1_callback, &comp_class);

  ros::Subscriber gripper_2_state_subscriber = node.subscribe(
    "/ariac/arm2/gripper/state", 10, 
    &MyCompetitionClass::gripper_2_callback, &comp_class);

  ROS_INFO("Setup complete.");
  start_competition(node);
  ros::spin();  // This executes callbacks on new data until ctrl-c.

  return 0;
}
// %EndTag(MAIN)%
// %EndTag(FULLTEXT)%
