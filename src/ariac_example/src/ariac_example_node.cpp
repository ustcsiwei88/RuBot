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
#include <cmath>

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

#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <trajectory_msgs/JointTrajectory.h>

#define PI_2 6.283185307179586
#define PI 3.141592653589793


// %EndTag(INCLUDE_STATEMENTS)%

// %Tag(START_COMP)%
/// Start the competition by waiting for and then calling the start ROS Service.

#define MAIN_FILE
using namespace std;

vector<double> kinematic(vector<double> theta);
vector<double> invkinematic(vector<double> pose);
vector<double> invkinematic_belt(vector<double> pose);

enum State{
  IDLE,
  BELT,
  FUMBLE,
  CLASSIFY,
  TRANSFER,
  TRANSIT,
  FAULTY,
  FLIP
};

enum PType{
  GASKET=1,
  GEAR=2,
  DISC=3,
  PISTON_ROD=4,
  PULLEY=5
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
class Shipment{
public:
  int priority;
  int agv;
  vector<pair<double,double>> position;
  vector<double> theta;
  vector<int> obj_t;
  vector<bool> flipped;
  vector<bool> finished;
  string shipment_t;
  Shipment(int priority=100):priority(priority){
  }
};
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

    agv_1 = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
    agv_2 = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");

    arm_1_joint.resize(6);
    arm_2_joint.resize(6);

    arm_1_joint_goal.resize(6);
    arm_2_joint_goal.resize(6);

    //idle = true;
    arm_1_state = IDLE;
    arm_2_state = IDLE;

    catched_1 = false;
    catched_2 = false;

    enabled_1 = false;
    enabled_2 = false;

    fum_1_init = true;
    fum_2_init = true;

    trans_1=false, trans_2=false;

    bin_type.resize(7,-1);    // -1 unknown, 0 empty, 1 disk, 2 gasket, 3 gear, 4 piston_rod, 5 pulley_part

    agv1_state = false;
    agv2_state = false;


    bin_mem.resize(6); //-1 unknown 0 empty 1 yes

    // for(auto i: classify_pos_2) cout<<i<<" ";
    //   cout<<endl;
    // for(auto i: classify2bpos_2) cout<<i<<" ";
    //   cout<<endl;
    cos_table.resize(400);
    double tmp = -0.34999999404;
    for(int i=0;i<400;i++){
      cos_table[i] = cos(tmp);
      tmp += 0.00175438600127;
    }

    flip_pose_1[3] += PI/2;
    flip_pose_1[4] = PI-flip_pose_1[0];
    flip_pose_2[3] += PI/2;
    flip_pose_2[4] = -flip_pose_2[0]; 
    
    // for(auto w: flip_pose_1) cout<<w<<" ";cout<<endl;
    // for(auto w: flip_pose_2) cout<<w<<" ";cout<<endl;

  }

  vector<double> cos_table;
  /// Called when a new message is received.
  // void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
  //   if (msg->data != current_score_)
  //   {
  //     ROS_INFO_STREAM("Score: " << msg->data);
  //   }
  //   current_score_ = msg->data;
  // }


  /// Called when a new message is received.
  // void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
  //   if (msg->data == "done" && competition_state_ != "done")
  //   {
  //     ROS_INFO("Competition ended.");
  //   }
  //   competition_state_ = msg->data;
  // }

  vector<Shipment> shipments_1, shipments_2;

  int bin_t2int(string s){
    if(s[1]=='n') return shipments_1.size()>shipments_2.size()? 2:1;
    if(s[3]=='1') return 1;
    return 2;
  }
  /// Called when a new Order message is received.
  void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    // Order tmp;
    //received_orders_.resize(received_orders_.size()+1);
    // Order * tmp;
    // if(order_msg->shipments->agv_id == string("any")){
    //   if(shipments_1.size()<shipments_2.size()){
    //     tmp = &
    //   }
    // }
    // Order & tmp = received_orders_[received_orders_.size()-1];
    for(auto & item:order_msg->shipments){
      Shipment *tmp;
      int n = bin_t2int(item.agv_id);
      if(n==1) {
        shipments_1.resize(shipments_1.size()+1);
        tmp = &shipments_1[shipments_1.size()-1];
        tmp->agv = 1;
      }
      else{
        shipments_2.resize(shipments_2.size()+1);
        tmp = &shipments_2[shipments_2.size()-1];
        tmp->agv = 2;
      }
      tmp->shipment_t=(item.shipment_type);
      
      for(auto & item1: item.products){
        // cout<<item1.type<<" "<<type2int(item1.type)<<endl;
        tmp->obj_t.push_back(type2int(item1.type));
        auto & item2=item1.pose;
        tmp->position.emplace_back(item2.position.x, item2.position.y);
        double x,y,z,w;
        x = item2.orientation.x;
        y = item2.orientation.y;
        z = item2.orientation.z;
        w = item2.orientation.w;
        if(n==1){
          tmp->theta.push_back(atan2(2*(z*w+y*x), 1-2*(z*z+y*y)));
          // cout<<" type, "<<item1.type<<"  angle "<<*tmp->theta.rbegin()<<endl;
        }
        else{
          tmp->theta.push_back(-atan2(2*(z*w+y*x), 1-2*(z*z+y*y)));
        }
        if(z*z+w*w-y*y-x*x < 0){
          tmp->flipped.push_back(true);
        }
        else tmp->flipped.push_back(false);
      }
      tmp->finished.resize(tmp->position.size(),false);

    }
    // for(auto &: order_msg->shipment)
    // received_orders_.push_back(*order_msg);
  }

  //stage:
  //idle
  //transit
  //transfer

  int des_1=-1, des_2=-1;

  int fail_1=0, fail_2=0;
  int ship_id_1, ship_id_2;
  // 50 HZ
  /// Called when a new JointState message is received.hip
  void arm_1_joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    arm_1_current_joint_states_ = *joint_state_msg;
    
    arm_1_joint[0] = arm_1_current_joint_states_.position[3]; //shoulder pan
    arm_1_joint[1] = arm_1_current_joint_states_.position[2]; //shoulder lift
    arm_1_joint[2] = arm_1_current_joint_states_.position[0]; //elbow
    arm_1_joint[3] = arm_1_current_joint_states_.position[4]; //wrist1
    arm_1_joint[4] = arm_1_current_joint_states_.position[5]; //wrist2
    arm_1_joint[5] = arm_1_current_joint_states_.position[6]; //wrist3
    arm_1_linear = arm_1_current_joint_states_.position[1]; //linear
    if (!arm_1_has_been_zeroed_) {
      arm_1_has_been_zeroed_ = true;
      ROS_INFO("Sending arm to zero joint positions...");
      send_arm_to_zero_state(arm_1_joint_trajectory_publisher_);
      return;
    }
    if(shipments_1.size()>0 && !agv1_state){
      bool f = true;
      for(bool i: shipments_1[0].finished){
        if(!i){f=false;break;}
      }
      if(f) {
        agv(1, shipments_1[0].shipment_t);
        // shipments_1.erase(shipments_1.begin());
      }
    }
    // cout<< arm_1_state<<endl;
    switch(arm_1_state){
      case IDLE:
        // send_arm_to_state( arm_1_joint_trajectory_publisher_, invkinematic(vector<double>{0.001,-1.05, -0.1}), 0.3, 1.18);break;
        while(!events.empty() && (ros::Time::now() - events[0].first).toSec() > 9){
          events.pop_front();
          // may move to another queue in the future.
        }
        if(count_1 == 0 && (!reached(arm_1_joint, arm_1_joint_goal) || fabs(arm_1_linear - arm_1_linear_goal) > 4e-3))
          break;
        if(flip_lock>0){
          arm_1_state = FLIP;
          bin_num_1=0;
          break;
        }
        if(trans_2){
          open_gripper(1);
          bin_num_1 = 0;
          if(count_1<2){
            send_arm_to_state_n(arm_1_joint_trajectory_publisher_, vector<vector<double>>{
              desk_hand_1_0, desk_hand_1_4, desk_hand_1_4, 
              desk_hand_1_5, desk_hand_1_6, desk_hand_1_6, 
              desk_hand_1_7, desk_hand_1_8, desk_hand_1_8,
              desk_hand_1_9, desk_hand_1_10, desk_hand_1_10, 
              desk_hand_1_11, desk_hand_1_12, desk_hand_1_12
            }, vector<double>{1.5 , 3.0 , 3.2 , 3.6 , 3.9 , 4.1 , 4.5 , 4.8 , 5.0 , 5.4 , 5.7 , 5.9 , 6.3 , 6.5 , 6.7},
            vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0});
            count_1+=2;
          }
          else{
            if(catched_1){
              arm_1_state=CLASSIFY;
              count_1=0;
              trans_2=false;
              send_arm_to_state( arm_1_joint_trajectory_publisher_, classify_pos_1, 0.3, 0);
              trans_lock=false;
            }
            count_1++;
            if(count_1>350){
              count_1=1;
              fail_1++;
              if(fail_1==3){
                trans_2=false;
                count_1=0;
                arm_1_state=IDLE;
                fail_1=0;
                trans_lock=false;
                send_arm_to_state( arm_1_joint_trajectory_publisher_, desk_hand_1_3, 0.4, 0);
              }
            }
          }
          break;
        }
        // if(!event.empty()){
        //   arm_1_state = BELT;
        //   break;
        // }
        if(shipments_1.size()>0){
          for(int i=0;i<shipments_1[0].obj_t.size();i++){
            int item = shipments_1[0].obj_t[i];
            if(shipments_1[0].finished[i]) continue;
            for(auto &tmp:events)
              if(tmp.second==item){
                arm_1_state=BELT;
                goto e1;
              }
          }
        }
        if(shipments_2.size()>0){
          for(int i=0;i<shipments_2[0].obj_t.size();i++){
            int item = shipments_2[0].obj_t[i];
            if(shipments_2[0].finished[i]) continue;
            for(auto &tmp:events)
              if(tmp.second==item){
                arm_1_state=BELT;
                goto e1;
              }
          }
        }
        if(shipments_1.size()>0){
          for(int i=0;i<shipments_1[0].obj_t.size();i++){
            int item = shipments_1[0].obj_t[i];
            if(shipments_1[0].finished[i]) continue;
            for(int j=4;j<=6;j++){
              if(bin_type[j]==item){
                bin_num_1=j;
                arm_1_state=FUMBLE;
                dx_1=-1;
                goto e1;
              }
            }
          }
        }
        if(shipments_2.size()>0){
          for(int i=0;i<shipments_2[0].obj_t.size();i++){
            int item = shipments_2[0].obj_t[i];
            if(shipments_2[0].finished[i]) continue;
            for(int j=4;j<=6;j++){
              if(bin_type[j]==item){
                bin_num_1=j;
                arm_1_state=FUMBLE;
                dx_1=-1;
                goto e1;
              }
            }
          }
        }
        if(bin_type[4]<0){
          bin_num_1 = 4;
          dx_1=-1;
          arm_1_state = FUMBLE;
        }
        else if(bin_type[5]<0){
          bin_num_1 = 5;
          dx_1=-1;
          arm_1_state = FUMBLE;
        }
        else if(bin_type[6]<0){
          bin_num_1 = 6;
          dx_1=-1;
          arm_1_state = FUMBLE;
        }
        
        e1:
        break;
      case BELT:
        if(count_1==0){
          int ind = -1;
          if(shipments_1.size()>0){
            for(int i=0;i<shipments_1[0].obj_t.size();i++){
              int item = shipments_1[0].obj_t[i];
              if(shipments_1[0].finished[i]) continue;
              for(int j=0;j<events.size();j++)
                if(item==events[j].second){
                  ind = j;
                  goto e3;
                }
            }
          }
          if(shipments_2.size()>0){
            for(int i=0;i<shipments_2[0].obj_t.size();i++){
              int item = shipments_2[0].obj_t[i];
              if(shipments_2[0].finished[i]) continue;
              for(int j=0;j<events.size();j++)
                if(item==events[j].second){
                  ind = j;
                  goto e3;
                }
            }
          }
          e3:
          if(ind==-1){
            arm_1_state=IDLE;
            count_1=0;
            break;
          }
          ros::Duration tmp = ros::Time::now() - events[ind].first;
          double ttc = 2.5;
          double dist = (tmp.toSec() + ttc) * belt_power/100 * maxBeltVel + 0.92 - 2.55 - 0.02;
          if(events[ind].second==3) dist -= 0.01;
          else if(events[ind].second==5) dist -= 0.04;
          // double dist = (tmp.toSec() + ttc) * belt_power/100 * maxBeltVel + 0.92 - 2.25 - 0.06;
          //double linear = 0;
          //if(fabs(dist)>0.1)
          // cout<<"event size "<<events.size()<<endl;
          
          send_arm_to_state_n(arm_1_joint_trajectory_publisher_, 
            vector<vector<double>>{
              invkinematic_belt(vector<double>{-0.92, 0.02, 0.04}), 
              invkinematic_belt(vector<double>{-0.92, 0.02, -0.0658})
            }, vector<double>{ttc*3/5, ttc}, 
            vector<double>{-dist + belt_power / 100 * maxBeltVel * ttc * 2/5, -dist});
          events.erase(events.begin()+ind);
          // cout<<"event size after del "<<events.size()<<endl;
          open_gripper(1);
          count_1++;
          break;
        }
        if(catched_1){
          // auto tmp = kinematic(arm_1_joint);
          // tmp[2] += 0.2;
          if(count_1<150){ 
            send_arm_to_state(arm_1_joint_trajectory_publisher_,
              rest_joints, 0.4,arm_1_linear);
            count_1+=200;
            break;
          }
          // if(bin_type[bin_num_1]<0){
          if(fabs(arm_1_joint[0])<1e-2 && fabs(arm_1_joint[1]+2)<1e-2 && fabs(arm_1_joint[2]+1) < 1e-2)
            arm_1_state = CLASSIFY,count_1=0;
          // if(count_1>150)
          //   arm_1_state = IDLE, count_1=0;
          bin_num_1 = 0;
          // count_1++;
          break;
        }
        count_1++;
        // cout<<count_1<<endl;
        if(count_1>150)
          arm_1_state = IDLE, count_1=0;
        break;
      case FUMBLE:
        //open_gripper(1);
        if(catched_1){
          // dx_1 = -1;
          dir_1= true;fum_1_init=true;
          auto tmp = kinematic(arm_1_joint);
          tmp[2] += 0.3;
          send_arm_to_state(arm_1_joint_trajectory_publisher_,
            invkinematic(tmp),0.4,arm_1_linear);
          // if(bin_type[bin_num_1]<0){
            arm_1_state = CLASSIFY;
          // }
        }
        if(trans_2 || flip_lock>0){
          arm_1_state = flip_lock>0? FLIP: IDLE;
          bin_mem[bin_num_1-1][make_pair(dx_1,dy_1)]=0;
          fum_1_init=true;
          break;
        }
        {
          bool tag=false;
          if(shipments_1.size()>0){
            for(int i=0;i<shipments_1[0].obj_t.size();i++){
              int item = shipments_1[0].obj_t[i];
              if(shipments_1[0].finished[i]) continue;
              for(auto &tmp:events)
                if(tmp.second==item){
                  tag=true;
                  goto e4;
                }
            }
          }
          if(shipments_2.size()>0){
            for(int i=0;i<shipments_2[0].obj_t.size();i++){
              int item = shipments_2[0].obj_t[i];
              if(shipments_2[0].finished[i]) continue;
              for(auto &tmp:events)
                if(tmp.second==item){
                  tag=true;
                  goto e4;
                }
            }
          }

          e4:
          if(tag){
    //        send_arm_to_state( arm_1_joint_trajectory_publisher_, rest_joints, 0.3, -0.3);
            arm_1_state = BELT;
            bin_mem[bin_num_1-1][make_pair(dx_1,dy_1)]=0;
            fum_1_init=true;
            break;
          }
        }
        if(reached(arm_1_joint, arm_1_joint_goal) && fabs(arm_1_linear - arm_1_linear_goal) <= 4e-3){
          open_gripper(1);
          if(!fumble(bin_num_1)){
            bin_type[bin_num_1]=0; //empty
            arm_1_state=IDLE;
          }
        }
        break;
      case CLASSIFY:
        if(count_1==0 && reached(arm_1_joint, arm_1_joint_goal) && fabs(arm_1_linear - arm_1_linear_goal) <= 4e-3){
          send_arm_to_state( arm_1_joint_trajectory_publisher_, classify_pos_1, 0.3, 0);
          count_1++;
        }
        if(reached(arm_1_joint, arm_1_joint_goal) && fabs(arm_1_linear - arm_1_linear_goal) <= 4e-3){
          count_1++;
          if(count_1==2){
            signal_1=false;
          }
          if(signal_1){   //use signal instead  //wait around 0.4s for classification
            bin_type[bin_num_1] = type_1;
            des_1 = -1;
            x_r_1 = divx_1;
            y_r_1 = divy_1;
            if(shipments_1.size()>0){
              for(int i=0;i<shipments_1[0].obj_t.size();i++){
                if(!shipments_1[0].finished[i] && type_1 == shipments_1[0].obj_t[i]){
                  
                  //real lock in the future, if getting into problem
                  if(flipped_1 == shipments_1[0].flipped[i]){           
                    arm_1_state = TRANSIT;
                    des_1 = 1;
                    count_1 = 0;
                    x_d_1 = shipments_1[0].position[i].first;
                    y_d_1 = shipments_1[0].position[i].second;
                    // cout<<shipments_1[0].theta[i] <<" ---=-=-=---- "<<theta_1<<endl;
                    dtheta_1 = shipments_1[0].theta[i] - theta_1;
                    // cout<<"theta_1 :"<<theta_1 <<" thetai"<<shipments_1[0].theta[i]<<endl;
                    while(dtheta_1 > PI) dtheta_1 -= PI_2;
                    while(dtheta_1 <= -PI) dtheta_1 += PI_2;

                    double tx_r_1 = cos(dtheta_1) * x_r_1 - sin(dtheta_1) * y_r_1;
                    double ty_r_1 = cos(dtheta_1) * y_r_1 + sin(dtheta_1) * x_r_1;
                    x_r_1 = tx_r_1;
                    y_r_1 = ty_r_1;
                    ship_id_1 = i;
                    break;
                  }
                }
              }
              if(des_1==1) break;
            }

            if(shipments_2.size()>0){
              for(int i=0;i<shipments_2[0].obj_t.size();i++){
                if(!shipments_2[0].finished[i] && type_1 == shipments_2[0].obj_t[i]){
                  if(flipped_1 != shipments_2[0].flipped[i]){
                    if(flip_lock==0){
                      flip_lock=1;
                      des_1 = 2;
                      count_1=0;
                      arm_1_state = FLIP;
                      break;
                    }
                  }
                }
              }
              if(des_1 ==2) break;
            }

            if(shipments_2.size()>0){
              for(int i=0;i<shipments_2[0].obj_t.size();i++){
                if(!shipments_2[0].finished[i] && type_1 == shipments_2[0].obj_t[i]){
                  if(flipped_1 == shipments_2[0].flipped[i]){
                    if(!trans_lock){
                      trans_lock=true;
                      des_1 = 2;
                      arm_1_state = TRANSIT;
                      count_1 = 0;
                      break;
                    }
                  }
                }
              }
              if(des_1 ==2) break;
            }

            if(shipments_1.size()>0){
              for(int i=0;i<shipments_1[0].obj_t.size();i++){
                if(!shipments_1[0].finished[i] && type_1 == shipments_1[0].obj_t[i]){
                  
                  //real lock in the future, if getting into problem
                  if(flipped_1 != shipments_1[0].flipped[i]){
                    if(flip_lock==0){
                      flip_lock=1;
                      arm_1_state = FLIP;
                      des_1 = 1;
                      count_1 = 0;
                      break;
                    }
                  }
                }
              }
              if(des_1==1) break;
            }

            //put it back
            des_1=3;
            arm_1_state = TRANSIT;
            count_1=0;
          }
        }
        break;
      case TRANSIT:
        if(!enabled_1)
          open_gripper(1);
        if(catched_1){
          // arm_1_state=TRANSFER;
          // auto start = kinematic(arm_1_joint);
          // start[2]+=0.17;

          if(des_1==1){

            // send_arm_to_state_n(arm_1_joint_trajectory_publisher_, 
            //   vector<vector<double>>{
            //     classify2bpos_1, 
            //     invkinematic(vector<double>{0.001+ /*x_r_1 - */x_d_1*3/4, -1.1+/*y_r_1-*/y_d_1*3/4, 0.2}), 
            //     invkinematic(vector<double>{0.001+ /*x_r_1 - */x_d_1*3/4, -1.1+/*y_r_1-*/y_d_1*3/4, 0.0})}, 
            //   vector<double>{1,2,3}, vector<double>{0.5 , 1.18, 1.18});

            // arm limitation
            if(-1.05 + y_d_1 - y_r_1 < -1.29) y_r_1 = 1.29-1.05 + y_d_1;
            auto p1 = invkinematic(vector<double>{0.00001 + x_d_1 - x_r_1, -1.05 + y_d_1 - y_r_1, -0.0});
            auto p2 = invkinematic(vector<double>{0.00001 + x_d_1 - x_r_1, -1.05 + y_d_1 - y_r_1, -0.1});
            p1[5] -= dtheta_1;
            // cout<<y_d_1 << " =-=-=-= "<<y_r_1<<endl;
            // for(auto w:vector<double>{0.00001 + x_d_1 - x_r_1, -1.05 + y_d_1 - y_r_1, 0.1}){
            //     cout<<w<<"----\n";
            // }
            // for(auto w:p1){
            //     cout<<w<<"===\n";
            // }
            while(p1[5]>PI) p1[5]-=PI_2;
            while(p1[5]<=-PI) p1[5]+=PI_2;
            p2[5]=p1[5];
            send_arm_to_state_n(arm_1_joint_trajectory_publisher_, 
              vector<vector<double>>{p1,p2},
              vector<double>{2,3}, vector<double>{1.18, 1.18});
            arm_1_state = TRANSFER;
          }
          else if(des_1==2){
            if(count_1==0){
              // if(type_1==PISTON_ROD){
              //   x_r_1*=0.8;
              //   y_r_1*=0.8;
              // }
              desk_hand_1_2_pose[0] -= x_r_1;
              desk_hand_1_1_pose[0] -= x_r_1;
              desk_hand_1_1_pose[1] -= y_r_1;
              desk_hand_1_2_pose[1] -= y_r_1;
              send_arm_to_state_n(arm_1_joint_trajectory_publisher_, 
                vector<vector<double>>{invkinematic(desk_hand_1_1_pose), invkinematic(desk_hand_1_2_pose)},
                vector<double>{0.5,0.8}, vector<double>{0,0});
              desk_hand_1_2_pose[0] += x_r_1;
              desk_hand_1_1_pose[0] += x_r_1;
              desk_hand_1_1_pose[1] += y_r_1;
              desk_hand_1_2_pose[1] += y_r_1;
              count_1++;
            }
            else{
              if(reached(arm_1_joint, arm_1_joint_goal) && fabs(arm_1_linear) <= 4e-3){
                close_gripper(1);
                send_arm_to_state_n(arm_1_joint_trajectory_publisher_, 
                  vector<vector<double>>{arm_1_joint_goal, desk_hand_1_3}, 
                  vector<double>{0.2,0.6}, 
                  vector<double>{0.0,0.0});
                count_1=0;
                arm_1_state = IDLE;
                trans_1=true;
                
              }
            }
          }
          else{//put back
            int i;
            for(i=6;i>0;i--){
              if(bin_type[i]==type_1) break;
            }
            if(i==0){
              for(i=6;i>0;i--){
                if(bin_type[i]==0) break;
              }
            }
            if(i==0){
              close_gripper(1);
              //send_arm_to_zero_state(arm_1_joint_trajectory_publisher_);/
              arm_1_state = IDLE;
              break;
            }
            else if(i>=4){
              double y = bin_y[i-1];
              double x = -0.4;
              double z = 0.64;
              double dx=0.16, dy=0;
              auto p1 = invkinematic(vector<double>{-x+dx, dy, z-0.9 + 0.6});
              // auto p2 = invkinematic(vector<double>{-x+dx, dy, z-0.9+0.01});
              auto p2 = invkinematic(vector<double>{-x+dx, dy, z-0.9 + 0.23});
              send_arm_to_state_n(arm_1_joint_trajectory_publisher_, vector<vector<double>>{
                p1, p2}, 
                vector<double>{0.6, 1}, 
                vector<double>{y - arm_1_zero, y - arm_1_zero});
              bin_mem[i-1][make_pair(dx,dy)]=0;
              arm_1_state = TRANSFER;
              bin_type[i] = type_1;
            }
            else{
              des_1=2;break;
              // if(count_1==0)
              //   send_arm_to_state_n(arm_1_joint_trajectory_publisher_, 
              //       vector<vector<double>>{desk_hand_1_1, desk_hand_1_2},
              //       vector<double>{0.5,0.8}, vector<double>{0,0}), count_1++;
              // else{
              //   if(reached(arm_1_joint, desk_hand_1_2) && fabs(arm_1_linear) <= 4e-3){
              //     close_gripper(1);
              //     trans_2=false;
              //     send_arm_to_state_n(arm_1_joint_trajectory_publisher_, 
              //       vector<vector<double>>{desk_hand_1_3, desk_hand},
              //       0.2, -0.1);
              //     count_1=0;
              //     arm_1_state = IDLE;
              //   }
              // }
              // bin_type[i]=type_1;
            }
          }
        }
        break;
      case TRANSFER:
        if(reached(arm_1_joint, arm_1_joint_goal) && fabs(arm_1_linear - arm_1_linear_goal) <= 4e-3){
          close_gripper(1);
          arm_1_state = IDLE;
          // send_arm_to_state(arm_2_joint_trajectory_publisher_, rest_joints, 
          //   0.5, 0);

          //wait 0.4 sec to drop
          if(des_1 == 1){
            if(count_1 == 20){
              if(faul_1){
                cout<<"caught faulty product"<<endl;
                arm_1_state = FAULTY;
                count_1=0;
                break;
              }
              shipments_1[0].finished[ship_id_1]=true;
              count_1=0;
              arm_1_state = IDLE;
            }
            else{
              count_1++;
              arm_1_state = TRANSFER;
            }
          }
          // send_arm_to_zero_state(arm_2_joint_trajectory_publisher_);
        }
        break;
      case FAULTY:
        if(count_1==0){
          if(reached(arm_1_joint, arm_1_joint_goal) && fabs(arm_1_linear - arm_1_linear_goal) <= 4e-3){
            open_gripper(1);
            auto p1 = invkinematic(vector<double>{0.00001 + faul_1_x, -1.05 + faul_1_y, -0.1});
            auto p2 = invkinematic(vector<double>{0.00001 + faul_1_x, -1.05 + faul_1_y, -0.23});
            auto p3 = invkinematic(vector<double>{0.00001 + faul_1_x, -1.05 + faul_1_y, -0.06});
            
            send_arm_to_state_n(
              arm_1_joint_trajectory_publisher_, 
              vector<vector<double>>{p1,p2,p2,p3},
              vector<double>{0.45, 0.9, 1.3, 1.7}, vector<double>{1.18,1.18,1.18,1.18}
              );
            count_1++;
          }
        }
        else if(count_1==1){
          if(reached(arm_1_joint, arm_1_joint_goal) && fabs(arm_1_linear - arm_1_linear_goal) <= 4e-3){
            if(catched_1){
              send_arm_to_state_n(
                arm_1_joint_trajectory_publisher_,
                vector<vector<double>>{throw_1_1, throw_1_2},
                vector<double>{0.4, 0.6}, vector<double>{0.9, 0.9}
                );
              count_1++;
              break;
            }
            count_1=0;
          }
        }
        else{
          if(reached(arm_1_joint, arm_1_joint_goal) && fabs(arm_1_linear - arm_1_linear_goal) <= 4e-3){
            close_gripper(1);
            count_1++;
            if(count_1==12){ // ~0.2 sec to throw it
              arm_1_state=IDLE;
              count_1=0;
            }
            break;
          }
        }
        break;
      case FLIP:
        if(count_1==0){
          send_arm_to_state(arm_1_joint_trajectory_publisher_, flip_pose_1, 0.5, -0.03);
          count_1++;
          break;
        }
        if(flip_lock == 1){
          if(flipok){
            flipok = false;
            flip_lock=0;
            // close_gripper(1);
            send_arm_to_state(arm_1_joint_trajectory_publisher_, flip_pose_1, 0.2, 0.05);
            arm_1_state = IDLE;
            count_1=0;
          }
        }
        else{
          if(catched_1){
            flipok = true;
            close_gripper(2);
            send_arm_to_state(arm_1_joint_trajectory_publisher_, flip_pose_1, 0.7, -0.05);
            arm_1_state = CLASSIFY;
            count_1=0;
            break;
          }
          if(count_1==1 && reached(arm_1_joint, flip_pose_1)){
            open_gripper(1);
            send_arm_to_state(arm_1_joint_trajectory_publisher_, flip_pose_1, 1.5, -0.28);
            count_1++;
          }
        }
        break;
    }


  }

  double dx_1 = -1;
  double dy_1 = 0.02;
  bool dir_1 = true;
  
  double dx_2 = -1;
  double dy_2 = 0.02;
  bool dir_2 = true;
  
  int count_1=0, count_2=0;

  vector<map<pair<double,double>, int>> bin_mem;

  bool fumble(int bin_num){
    bin_num--;
    double y = bin_y[bin_num];
    double x = -0.4;
    double z = 0.64;
    // cout<<"fumbling No."<<bin_num<<endl;
    double dx,dy;
    bool dir;
    if(bin_num>=3){
      refum1:
      dx = dx_1, dy=dy_1;dir=dir_1;
      if(dx<-0.5){
        dx=-0.08; dy=-0.24; dir=true; fum_1_init=true;
      }
      else{
        dx += dir ? 0.06 : -0.06;
        //dy += 0.05;
        if(dx > 0.479){dx=0.46, dy+=0.06;dir=!dir;}
        else if(dx<-0.081){dx=-0.08, dy+=0.06;dir=!dir;}
      }
      if(dy>0.241){dx=-1; dir_1=true; dx_1=-1;return false;}
      dx_1=dx;dy_1=dy;dir_1=dir;
      if(bin_mem[bin_num][make_pair(dx,dy)]>0){
        goto refum1;
      }
      bin_mem[bin_num][make_pair(dx,dy)]=1;
    }
    else{
      refum2:
      dx = dx_2, dy=dy_2;dir=dir_2;
      if(dx<-.5){
        dx=-0.08; dy=-0.24; dir=true; fum_2_init=true;
      }
      else{
        dx += dir ? 0.06 : -0.06;
        //dy += 0.05;
        if(dx > 0.479){dx=0.46, dy+=0.06;dir=!dir;}
        else if(dx<-0.081){dx=-0.08, dy+=0.06;dir=!dir;}
      }
      if(dy>0.241){dx=-1; dir_2=true; dx_2=-1;return false;}
      dx_2=dx;dy_2=dy;dir_2=dir;
      if(bin_mem[bin_num][make_pair(dx,dy)]>0){
        goto refum2;
      }
      bin_mem[bin_num][make_pair(dx,dy)]=1;
    }
    // cout<<dir<<endl;
    // cout<<dx<<" "<<dy<<endl;
    if(bin_num>=3){
      vector<double> p1;
      if(fum_1_init)p1 = invkinematic(vector<double>{-x+0.19, -0.03, z-0.9 + 0.6});
      else 
        p1 = invkinematic(vector<double>{-x+dx, dy, z-0.9+0.13});
      auto p2 = invkinematic(vector<double>{-x+dx, dy, z-0.9-0.011});
      auto p3 = invkinematic(vector<double>{-x+dx, dy, z-0.9+0.16});
      // auto res = kinematic(p1);
      //auto p1 = invkinematic(vector<double>{-x+dx, dy, z+0.05});
      if(fum_1_init)
        send_arm_to_state_n(arm_1_joint_trajectory_publisher_, vector<vector<double>>{p1,p2,p2,p3}, 
          vector<double>{1.25, 1.55, 1.8, 2}, 
          vector<double>{y - arm_1_zero, y - arm_1_zero, y - arm_1_zero, y - arm_1_zero});
      else
        send_arm_to_state_n(arm_1_joint_trajectory_publisher_, vector<vector<double>>{p1,p2,p2,p3}, 
          vector<double>{0.2, 0.5, 0.75, 0.95}, 
          vector<double>{y - arm_1_zero, y - arm_1_zero, y - arm_1_zero, y - arm_1_zero});
      fum_1_init=false;
    }

    else{
      vector<double> p1;
      if(fum_2_init)p1 = invkinematic(vector<double>{-x+0.19, -0.03, z-0.9 + 0.6});
      else 
        p1 = invkinematic(vector<double>{-x+dx, dy, z-0.9+0.13});
      auto p2 = invkinematic(vector<double>{-x+dx, dy, z-0.9-0.010});
      auto p3 = invkinematic(vector<double>{-x+dx, dy, z-0.9+0.16});
      // auto res = kinematic(p1);
      //auto p1 = invkinematic(vector<double>{-x+dx, dy, z+0.05});
      if(fum_2_init)
        send_arm_to_state_n(arm_2_joint_trajectory_publisher_, vector<vector<double>>{p1,p2,p2,p3}, 
          vector<double>{1.0, 1.5, 1.6, 2},
           vector<double>{y - arm_2_zero, y - arm_2_zero, y - arm_2_zero, y - arm_2_zero});
      else
        send_arm_to_state_n(arm_2_joint_trajectory_publisher_, vector<vector<double>>{p1,p2,p2,p3}, 
          vector<double>{0.2, 0.6, 0.8, 0.9}, 
          vector<double>{y - arm_2_zero, y - arm_2_zero, y - arm_2_zero, y - arm_2_zero});
      fum_2_init=false;
    }
    return true;

  }
  
  bool reached(const vector<double>&s, const vector<double>& t){
    double sum=0;
    for(int i=0;i<5/*6*/;i++){          // Gave up theta will do it if have time
      sum += min(fabs(s[i]-t[i]), fabs(6.28318530718 - fabs(s[i]-t[i])));
    }
    //cout<<sum<<endl;
    return sum < 3e-2;
  }

  void arm_2_joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Joint States arm 2 (throttled to 0.1 Hz):\n" << *joint_state_msg);
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
      send_arm_to_zero_state(arm_2_joint_trajectory_publisher_);
      return;
    }

    // cout<<arm_2_state<<"---"<<endl;
    // for(double ii: arm_2_joint) cout<< ii<<" ";
    //       cout<<endl;
    //     for(double ii: arm_2_joint_goal) cout<< ii<<" ";
    //       cout<<endl;
    if(shipments_2.size()>0 && !agv2_state){
      bool f = true;
      for(bool i: shipments_2[0].finished){
        if(!i){f=false;break;}
      }
      if(f) {
        agv(2,shipments_2[0].shipment_t);
        // shipments_2.erase(shipments_2.begin());
      }
    }
    switch(arm_2_state){
      case IDLE:
        // cout<<"arm 2 idling"<<endl;
        // send_arm_to_state( arm_2_joint_trajectory_publisher_, invkinematic(vector<double>{0.001, 1.05, -0.1}), 0.3, -1.18);break;
        if(count_2==0 && (!reached(arm_2_joint, arm_2_joint_goal) || fabs(arm_2_linear - arm_2_linear_goal) > 4e-3))
          break;
        if(flip_lock>0){
          arm_2_state = FLIP;
          bin_num_2=0;
          break;
        }
        if(trans_1){
          open_gripper(2);
          bin_num_2 = 0;

          // cout<<"fetching from desk"<<endl;
          if(count_2<2){
            send_arm_to_state_n(arm_2_joint_trajectory_publisher_, vector<vector<double>>{
              desk_hand_2_0, desk_hand_2_4, desk_hand_2_4, 
              desk_hand_2_5, desk_hand_2_6, desk_hand_2_6, 
              desk_hand_2_7, desk_hand_2_8, desk_hand_2_8,
              desk_hand_2_9, desk_hand_2_10, desk_hand_2_10, 
              desk_hand_2_11, desk_hand_2_12, desk_hand_2_12
            }, vector<double>{1.5 , 3.0 , 3.2 , 3.6 , 3.9 , 4.1 , 4.5 , 4.8 , 5.0 , 5.4 , 5.7 , 5.9 , 6.3 , 6.5 , 6.7}, 
            vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0});
            count_2+=2;
          }
          else{
            if(catched_2){
              send_arm_to_state( arm_2_joint_trajectory_publisher_, classify_pos_2, 0.3, 0);
              arm_2_state=CLASSIFY;
              count_2=0;
              trans_1=false;
              trans_lock=false;
            }
            count_2++;
            if(count_2>350){
              count_2=1;
              fail_2++;
              if(fail_2==3){
                trans_1=false;
                trans_lock=false;
                count_2=0;
                arm_2_state=IDLE;
                fail_2=0;
                send_arm_to_state(arm_2_joint_trajectory_publisher_, desk_hand_2_3, 0.4, 0.0);
              }
              break;
            }
          }
          break;
        }

        if(shipments_2.size()>0){
          for(int i=0;i<shipments_2[0].obj_t.size();i++){
            int item = shipments_2[0].obj_t[i];
            if(shipments_2[0].finished[i]) continue;
            for(int j=1;j<=3;j++){
              if(bin_type[j]==item){
                bin_num_2=j;
                arm_2_state=FUMBLE;
                dx_2=-1;
                goto e2;
              }
            }
          }
        }
        if(shipments_1.size()>0){
          for(int i=0;i<shipments_1[0].obj_t.size();i++){
            int item = shipments_1[0].obj_t[i];
            if(shipments_1[0].finished[i]) continue;
            for(int j=1;j<=3;j++){
              if(bin_type[j]==item){
                bin_num_2 = j;
                arm_2_state=FUMBLE;
                dx_2=-1;
                goto e2;
              }
            }
          }
        }

        if(bin_type[1]<0){
          bin_num_2 = 1;
          dx_2=-1;
          arm_2_state = FUMBLE;
        }
        else if(bin_type[2]<0){
          bin_num_2 = 2;
          dx_2=-1;
          arm_2_state = FUMBLE;
        }
        else if(bin_type[3]<0){
          bin_num_2 = 3;
          dx_2=-1;
          arm_2_state = FUMBLE;
        }
        
        e2:
        // if(!event.empty()){
        //   ros::Duration tmp = ros::Time::now() - event[0];
        //   double ttc = 1.5;
        //   double dist = (tmp.toSec() + ttc) * belt_power/100 * maxBeltVel + 0.92 - 2.25 ;
        //   //double linear = 0;
        //   //if(fabs(dist)>0.1)
        //   send_arm_to_state_n(arm_1_joint_trajectory_publisher_, 
        //     vector<vector<double>>{
        //       invkinematic(vector<double>{-0.92, -0.02, 0.03}), 
        //       invkinematic(vector<double>{-0.92, -0.02, -0.07})
        //     }, vector<double>{ttc*3/4, ttc}, vector<double>{-dist + belt_power / 100 * maxBeltVel * ttc / 4, -dist});
        //   event.pop_front();
        //   arm_2_state = TRANSIT;
        // }
        break;
      case FUMBLE:
        //open_gripper(2);
        if(catched_2){
          dx_2 = -1;
          dir_2= true;fum_2_init=true;
          auto tmp = kinematic(arm_2_joint);
          tmp[2] += 0.3;
          send_arm_to_state(arm_2_joint_trajectory_publisher_,
            invkinematic(tmp),0.4,arm_2_linear);
          // if(bin_type[bin_num_2]<0){
            arm_2_state = CLASSIFY;
          // }
          bin_mem[bin_num_2-1][make_pair(dx_2,dy_2)]=0;
          break;
        }
        if(trans_1 || flip_lock>0){
          arm_2_state = flip_lock>0? FLIP : IDLE;
          fum_2_init=true;
          bin_mem[bin_num_2-1][make_pair(dx_2,dy_2)]=0;
          break;
        }
        if(reached(arm_2_joint, arm_2_joint_goal) && fabs(arm_2_linear - arm_2_linear_goal) <= 4e-3){
          open_gripper(2);
          if(!fumble(bin_num_2)){
            bin_type[bin_num_2]=0; //empty
            arm_2_state=IDLE;
          }
        }
        break;
      case CLASSIFY:
        if(count_2==0 && reached(arm_2_joint, arm_2_joint_goal) && fabs(arm_2_linear - arm_2_linear_goal) <= 4e-3){
          send_arm_to_state( arm_2_joint_trajectory_publisher_, classify_pos_2, 0.3, 0);
          count_2++;
        }
        
        if(reached(arm_2_joint, arm_2_joint_goal) && fabs(arm_2_linear - arm_2_linear_goal) <= 4e-3){
          count_2++;
          if(count_2==2){
            signal_2=false;
          }
          if(signal_2){   //use signal instead  //wait around 0.4s for classification
            bin_type[bin_num_2] = type_2;
            des_2 = 0;
            x_r_2 = divx_2;
            y_r_2 = divy_2;
            if(shipments_2.size()>0){

              for(int i=0;i<shipments_2[0].obj_t.size();i++){
                if(!shipments_2[0].finished[i] && type_2 == shipments_2[0].obj_t[i]){
                  // cout<<shipments_2[0].obj_t[i]<<"----------type matched"<<endl;

                  if(flipped_2 == shipments_2[0].flipped[i]){
                    arm_2_state = TRANSIT;
                    des_2 = 1;
                    count_2 = 0;
                    x_d_2 = shipments_2[0].position[i].first;
                    y_d_2 = shipments_2[0].position[i].second;
                    
                    dtheta_2 = shipments_2[0].theta[i] - theta_2;
                    while(dtheta_2 > PI) dtheta_2 -= PI_2;
                    while(dtheta_2 <= -PI) dtheta_2 += PI_2;

                    double tx_r_2 = cos(dtheta_2) * x_r_2 + sin(dtheta_2) * y_r_2;
                    double ty_r_2 = cos(dtheta_2) * y_r_2 - sin(dtheta_2) * x_r_2;
                    x_r_2 = tx_r_2;
                    y_r_2 = ty_r_2;

                    ship_id_2 = i;
                    break;
                  }
                }
              }
              if(des_2==1) break;
            }

            if(shipments_1.size()>0){
              for(int i=0;i<shipments_1[0].obj_t.size();i++){
                if(!shipments_1[0].finished[i] && type_2 == shipments_1[0].obj_t[i]){
                  if(flipped_2 != shipments_1[0].flipped[i]){
                    if(flip_lock==0){
                      flip_lock=2;
                      arm_2_state=FLIP;
                      count_2=0;
                      des_2=2;
                      break;
                    }
                  }
                }
              }
              if(des_2==2) break;
            }

            if(shipments_1.size()>0){
              for(int i=0;i<shipments_1[0].obj_t.size();i++){
                if(!shipments_1[0].finished[i] && type_2 == shipments_1[0].obj_t[i]){
                  if(flipped_2 == shipments_1[0].flipped[i]){
                    if(!trans_lock){
                      trans_lock=true;
                      des_2=2;
                      arm_2_state=TRANSIT;
                      count_2=0;
                      break;
                    }
                  }
                }
              }
              if(des_2==2) break;
            }


            if(shipments_2.size()>0){

              for(int i=0;i<shipments_2[0].obj_t.size();i++){
                if(!shipments_2[0].finished[i] && type_2 == shipments_2[0].obj_t[i]){
                  // cout<<shipments_2[0].obj_t[i]<<"----------type matched"<<endl;
                  if(flipped_2 != shipments_2[0].flipped[i]){
                    if(flip_lock==0){
                      flip_lock=2;
                      arm_2_state=FLIP;
                      des_2 = 1;
                      count_2 = 0;
                      break;
                    }
                  }
                }
              }
              if(des_2==1) break;
            }
            des_2=3;
            arm_2_state = TRANSIT;
            count_2=0;
          }
        }
        break;
      case TRANSIT:
        if(!enabled_2)
          open_gripper(2);
        if(catched_2){
          // arm_2_state=TRANSFER;
          //auto start = kinematic(arm_2_joint);
          //start[2]+=0.17;
          // cout<<"des2 is "<< des_2<<endl;
          if(des_2==1){            
            if(1.05 - y_d_2 + y_r_2 >1.29) y_r_2 = -1.05+1.29+y_d_2;
            auto p1 = invkinematic(vector<double>{0.00001 - x_d_2 - x_r_2, 1.05 - y_d_2 + y_r_2, -0.0});
            auto p2 = invkinematic(vector<double>{0.00001 - x_d_2 - x_r_2, 1.05 - y_d_2 + y_r_2, -0.1}); 
            p1[5] -= dtheta_2;
            while(p1[5]>PI) p1[5]-=PI_2;
            while(p1[5]<=-PI) p1[5]+=PI_2;
            p2[5]=p1[5];
            send_arm_to_state_n(arm_2_joint_trajectory_publisher_, 
              vector<vector<double>>{p1,p2},
              vector<double>{2, 3}, vector<double>{-1.18, -1.18});
            arm_2_state=TRANSFER;
          }
          else if(des_2==2){
            if(count_2==0){
              // send_arm_to_state_n(arm_2_joint_trajectory_publisher_, 
              //   vector<vector<double>>{desk_hand_2_1, desk_hand_2_2},
              //   vector<double>{0.5,0.8}, vector<double>{0,0});
              // if(type_2==PISTON_ROD){
              //   x_r_2*=0.8;
              //   y_r_2*=0.8;
              // }
              desk_hand_2_2_pose[0] -= x_r_2;
              desk_hand_2_1_pose[0] -= x_r_2;
              desk_hand_2_1_pose[1] += y_r_2;
              desk_hand_2_2_pose[1] += y_r_2;
              send_arm_to_state_n(arm_2_joint_trajectory_publisher_, 
                vector<vector<double>>{invkinematic(desk_hand_2_1_pose), invkinematic(desk_hand_2_2_pose)},
                vector<double>{0.5,0.8}, vector<double>{0,0});
              desk_hand_2_2_pose[0] += x_r_2;
              desk_hand_2_1_pose[0] += x_r_2;
              desk_hand_2_1_pose[1] -= y_r_2;
              desk_hand_2_2_pose[1] -= y_r_2;
              count_2++;
            }
            else{
              if(reached(arm_2_joint, arm_2_joint_goal) && fabs(arm_2_linear) <= 4e-3){
                close_gripper(2);
                send_arm_to_state_n(arm_2_joint_trajectory_publisher_, 
                  vector<vector<double>>{arm_2_joint_goal, desk_hand_2_3}, 
                  vector<double>{0.2, 0.6}, 
                  vector<double>{0.0, 0.0});
                count_2=0;
                trans_2=true;
                arm_2_state = IDLE;
              }
            }
          }
          else{   //put it back
            int i;
            for(i=1;i<7;i++){
              if(bin_type[i]==type_2) break;
            }
            // cout<<i<<endl;
            if(i==7){
              for(i=1;i<7;i++){
                if(bin_type[i]==0) break;
              }
            }
            if(i==7){
              close_gripper(2);
              // send_arm_to_zero_state(arm_2_joint_trajectory_publisher_);
              arm_2_state = IDLE;
            }
            else if(i<4){
              double y = bin_y[i-1];
              double x = -0.4;
              double z = 0.64;
              double dx=0.16, dy=0;
              auto p1 = invkinematic(vector<double>{-x+dx, dy, z-0.9 + 0.6});
              auto p2 = invkinematic(vector<double>{-x+dx, dy, z-0.9 + 0.23});
              send_arm_to_state_n(arm_2_joint_trajectory_publisher_, vector<vector<double>>{
                p1,p2}, 
                vector<double>{0.6, 1}, 
                vector<double>{y - arm_2_zero, y - arm_2_zero});
              arm_2_state = TRANSFER;
              bin_type[i] = type_2;
              bin_mem[i-1][make_pair(dx,dy)]=0;
            }
            else{
              des_2=2;break;
              // if(count_2==0)
              //   send_arm_to_state_n(arm_2_joint_trajectory_publisher_, 
              //       vector<vector<double>>{desk_hand_2_2, desk_hand_2_2},
              //       vector<double>{0.5,0.8}, vector<double>{0,0}), count_2++;
              // else{
              //   if(reached(arm_2_joint, desk_hand_2_2) && fabs(arm_2_linear) <= 4e-3){
              //     trans_2 = false;
              //     close_gripper(2);
              //     send_arm_to_state(arm_2_joint_trajectory_publisher_, desk_hand_2_3, 0.2, -0.1);
              //     count_2=0;
              //     arm_2_state = IDLE;
              //   }
              // }
              // bin_type[i]=type_2;
            }
          }
        }
        break;
      case TRANSFER:
        if(reached(arm_2_joint, arm_2_joint_goal) && fabs(arm_2_linear - arm_2_linear_goal) <= 4e-3){
          close_gripper(2);
          arm_2_state = IDLE;

          if(des_2 == 1){
            if(count_2 == 20){
              if(faul_2){
                cout<<"caught faulty product"<<endl;
                arm_2_state = FAULTY;
                count_2=0;
                break;
              }
              shipments_2[0].finished[ship_id_2]=true;
              count_2=0;
              arm_2_state=IDLE;
            }
            else{
              count_2++;
              arm_2_state=TRANSFER;
            }
          }
          // send_arm_to_state(arm_2_joint_trajectory_publisher_, rest_joints, 
          //   0.5, 0);
        }
        break;
      case FAULTY:
        if(count_2==0){
          if(reached(arm_2_joint, arm_2_joint_goal) && fabs(arm_2_linear - arm_2_linear_goal) <= 4e-3){
            open_gripper(2);
            auto p1 = invkinematic(vector<double>{0.00001 + faul_2_x, 1.05 - faul_2_y, -0.1});
            auto p2 = invkinematic(vector<double>{0.00001 + faul_2_x, 1.05 - faul_2_y, -0.23});
            auto p3 = invkinematic(vector<double>{0.00001 + faul_2_x, 1.05 - faul_2_y, -0.06});
            
            send_arm_to_state_n(
              arm_2_joint_trajectory_publisher_, 
              vector<vector<double>>{p1,p2,p2,p3},
              vector<double>{0.45, 0.9, 1.3, 1.7}, vector<double>{-1.18,-1.18,-1.18,-1.18}
              );
            count_2++;
          }
        }
        else if(count_2==1){
          if(reached(arm_2_joint, arm_2_joint_goal) && fabs(arm_2_linear - arm_2_linear_goal) <= 4e-3){
            if(catched_2){
              send_arm_to_state_n(
                arm_2_joint_trajectory_publisher_,
                vector<vector<double>>{throw_2_1, throw_2_2},
                vector<double>{0.4, 0.6}, vector<double>{-0.9, -0.9}
                );
              count_2++;
              break;
            }
            count_2=0;
          }
        }
        else{
          if(reached(arm_2_joint, arm_2_joint_goal) && fabs(arm_2_linear - arm_2_linear_goal) <= 4e-3){
            close_gripper(2);
            count_2++;
            if(count_2==12){ // ~0.2 sec to throw it
              arm_2_state=IDLE;
              count_2=0;
            }
            break;
          }
        }
        break;
      case FLIP:
        if(count_2==0){
          send_arm_to_state(arm_2_joint_trajectory_publisher_, flip_pose_2, 0.5, 0.03);
          count_2++;
          break;
        }
        if(flip_lock==2){
          if(flipok){
            flipok=false;
            flip_lock=0;
            // close_gripper(2);
            send_arm_to_state(arm_2_joint_trajectory_publisher_, flip_pose_2, 0.2, -0.05);
            arm_2_state=IDLE;
            count_2=0;
          }
        }
        else{
          if(catched_2){
            flipok=true;
            close_gripper(1);
            send_arm_to_state(arm_2_joint_trajectory_publisher_, flip_pose_2, 0.7, 0.05);
            arm_2_state = CLASSIFY;
            count_2=0;
            break;
          }
          if(count_2==1 && reached(arm_2_joint, flip_pose_2)){
            open_gripper(2);
            send_arm_to_state(arm_2_joint_trajectory_publisher_, flip_pose_2, 1.5, 0.28);
            count_2++;
          }
        }
        break;
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
    // msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    msg.points[0].positions = rest_joints;
    msg.points[0].positions.push_back(0);
    if(joint_trajectory_publisher == arm_1_joint_trajectory_publisher_){
      arm_1_linear_goal = 1.0, arm_1_joint_goal=rest_joints, close_gripper(1);msg.points[0].positions[6]=1.0;
    }
    else{
      arm_2_linear_goal = -1.0, arm_2_joint_goal=rest_joints, close_gripper(2);msg.points[0].positions[6]=-1.0;
    }
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(0.5);
    // ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
  }



  void send_arm_to_state(ros::Publisher & joint_trajectory_publisher, std::vector<double> joints, double t, double linear=0.0) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;

    if(joint_trajectory_publisher == arm_1_joint_trajectory_publisher_)
      arm_1_joint_goal = joints, arm_1_linear_goal = linear;
    else
      arm_2_joint_goal = joints, arm_2_linear_goal = linear;
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
    // ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
  }  

  void send_arm_to_state_2(ros::Publisher & joint_trajectory_publisher, std::vector<double> joints1, std::vector<double> joints2, double t, double linear=0.0) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;

    if(joint_trajectory_publisher == arm_1_joint_trajectory_publisher_)
      arm_1_joint_goal = joints2, arm_1_linear_goal = linear;
    else
      arm_2_joint_goal = joints2, arm_2_linear_goal = linear;
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

    // cout<<"got here"<<endl;
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(ros::Duration(t).toSec()*3/4);
    msg.points[1].time_from_start = ros::Duration(t);
    // ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
  }

  void send_arm_to_state_n(ros::Publisher & joint_trajectory_publisher, vector<std::vector<double>>  joints_l, vector<double>  t, vector<double>  linear) {
    trajectory_msgs::JointTrajectory msg;

    if(joint_trajectory_publisher == arm_1_joint_trajectory_publisher_)
      arm_1_joint_goal = *joints_l.rbegin(), arm_1_linear_goal = *linear.rbegin();
    else
      arm_2_joint_goal = *joints_l.rbegin(), arm_2_linear_goal = *linear.rbegin();
    
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
    

    msg.points.resize(joints_l.size());
    
    for(int i=0;i<joints_l.size();i++){
      msg.points[i].positions.resize(msg.joint_names.size());
      for(int j=0;j<6;j++){
        msg.points[i].positions[j] = joints_l[i][j];
      }
      msg.points[i].positions[6] = linear[i];
      msg.points[i].time_from_start = ros::Duration(t[i]);
    }
    // msg.points.resize(joints_l.size()*2+1);
    
    // for(int i=0;i<joints_l.size();i++){
    //   msg.points[2*i].positions.resize(msg.joint_names.size());
    //   for(int j=0;j<6;j++){
    //     msg.points[2*i].positions[j] = joints_l[i][j];
    //   }
    //   msg.points[2*i].positions[6] = linear[i];
    //   msg.points[2*i].time_from_start = ros::Duration(t[i]);
    //   if(i!=joints_l.size()-1){
    //     msg.points[2*i+1].positions.resize(msg.joint_names.size());
    //     for(int j=0;j<6;j++){
    //       msg.points[2*i+1].positions[j] = (joints_l[i][j]+joints_l[i+1][j])/2;
    //     }
    //     msg.points[2*i+1].positions[6] = (linear[i]+linear[i+1])/2;
    //     msg.points[2*i+1].time_from_start = ros::Duration((t[i]+t[i+1])/2);
    //   }
    // }

    // How long to take getting to the point (floating point seconds).
    // if(joint_trajectory_publisher == arm_2_joint_trajectory_publisher_) 
    //   ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
  }
  // %EndTag(ARM_ZERO)%

  /// Called when a new LogicalCameraImage message is received.
  void logical_camera_callback(
    const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    // ROS_INFO_STREAM_THROTTLE(10,
    //    "Logical camera: '" << image_msg->models.size() << "' objects.");
    //cout<<type_1<<type_2<<" "<<des_2<<endl;
    for(auto item: image_msg->models){
      // cout << item.pose.position<<endl;    // x -> z , y->y, z -> -x
      // cout << item.pose.orientation<<endl;
      // for(auto w:{item.pose.orientation.x,item.pose.orientation.y,
      //   item.pose.orientation.z, item.pose.orientation.w}){
      //   cout<<w<<" ^^ ";
      // }
      // cout<<endl;
      tf2::Quaternion tmp1(item.pose.orientation.x,item.pose.orientation.y,
        item.pose.orientation.z, item.pose.orientation.w);
      // tf2::Quaternion tmp2(image_msg->pose.orientation.x, image_msg->pose.orientation.y, 
      //   image_msg->pose.orientation.z, image_msg->pose.orientation.w);
      auto ori = q_logical*tmp1;
      ori.normalize();
      // auto & ori = item.pose.orientation;
      double x = ori.x(), y= ori.y(), z = ori.z(), w = ori.w();
      // cout<<x<<" "<<y<<" "<<z<<" "<<w<<endl;
      //cout << atan2(2*(x*y + z*w), 1-2*(y*y+z*z))<<" ";
      //cout << asin(2*(x*z-y*w))<<" ";
      //cout << atan2(2*(x*w+y*z), 1-2*(z*z+w*w))<<" "<<endl;

      if(item.pose.position.y > 0) {
        divx_1 = item.pose.position.z + 0.05;
        divy_1 = - item.pose.position.y + 0.17;
        theta_1 = PI + atan2(2*(z*w+y*x), 1-2*(z*z+y*y));
        if(theta_1>PI) theta_1-=PI_2;
        // theta_1 = - theta_1;
        // cout<<theta_1<<"---------------"<<endl;
        if(1-2*(x*x+y*y) < 0) flipped_1=true;
        else flipped_1=false;
        type_1 = type2int(item.type);
        signal_1=true;
      }
      else{
        divx_2 = item.pose.position.z + 0.05;
        divy_2 = item.pose.position.y + 0.17;
        theta_2 = atan2(2*(z*w+y*x), 1-2*(z*z+y*y));
        if(1-2*(x*x+y*y) < 0) flipped_2=true;
        else flipped_2=false;
        type_2 = type2int(item.type);
        signal_2=true;
      }
    }
    // int i = 0;
    // for(auto &item: image_msg->models){
    //   ROS_INFO_STREAM_THROTTLE(10+i,
    //   " item: " << ++i <<": " << item << "----");
    // }
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "\n");
  }
  double divx_1=0, divy_1=0, theta_1=0;
  double divx_2=0, divy_2=0, theta_2=0;
  bool flipped_1 = false, flipped_2 = false;
  int type_1=0;
  int type_2=0;
  int type2int(const string &type){
    if(type[0]=='g'){
      if(type[1]=='a') return 1;
      return 2;
    }
    else if(type[0]=='d') return 3;
    else{
      if(type[1]=='i') return 4;
      return 5;
    }
  }
  // void break_beam_callback(const osrf_gear::Proximity::ConstPtr & msg) {
  //   if (msg->object_detected) {  // If there is an object in proximity.
  //     ROS_INFO("Break beam triggered.");
  //     event.emplace_back(ros::Time::now());
  //   }
  // }
  double height = 0.0;
  ros::Time stime;
  void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
    // size_t number_of_valid_ranges = std::count_if(
    //   msg->ranges.begin(), msg->ranges.end(), [](const float f) {return std::isfinite(f);});
    // if (number_of_valid_ranges > 0) {
    //   ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
    // }

    // for(int i=0;i<msg->ranges.size();i++){
    //   if(msg->ranges[i] != HUGE_VAL)
    //     cout<<max(0.685 - msg->ranges[i] * cos_table[i],0)<<" ";
    // }
    
    double tmp = 0.0;
    for(int i=0;i<msg->ranges.size();i++){
      if(msg->ranges[i] != HUGE_VAL)
        tmp = max(tmp, 0.685 - msg->ranges[i] * cos_table[i]);
      // 0 empty, <0.005 piston, <0.01 gear, gasket<0.016, <0.021 disk, > 0.065 pully
    }
    if(height==0.0){
      if(tmp>0){
        height=tmp;
        stime = msg->header.stamp;
        height=tmp;
      }
    }
    else if(tmp==0.0){
      if(height>0){
        PType type = PULLEY;
        if(height<0.005) type = PISTON_ROD;  //piston
        else if(height<0.01) type = GEAR;  //gear
        else if(height<0.016) type = GASKET; //gasket
        else if(height<0.025) type = DISC; //disc
        else type = PULLEY;
        events.emplace_back(stime, type);
         // cout<<type<<"--"<<endl;
        
      }
      height = 0.0;
    }
    else{
      height = max(height, tmp);
    }
  }

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

    // if(!srv.response.success){
    //   ROS_ERROR_STREAM("Gripper Failed");
    // }
    // else{
    //   // ROS_INFO("Gripper openned");
    // }
  }

  void agv(int num, string ship_t){
    osrf_gear::AGVControl srv;
    srv.request.shipment_type = ship_t;
    // srv.request.shipment_type = string("order_") + to_string(order) + string("_shipment_") + to_string(kit);
    if(num==1){
      agv_1.call(srv);
      agv1_state=true;
    }
    else{
      agv_2.call(srv);
      agv2_state=true;
    }
  }

  void gripper_1_callback(const osrf_gear::VacuumGripperState::ConstPtr & msg){
    catched_1 = msg->attached;
    enabled_1 = msg->enabled;
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
    enabled_2 = msg->enabled;
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

  void agv_1_callback(std_msgs::String::ConstPtr &msg){
    if(agv1_state){
      if(msg->data[0]=='r' && msg->data[2]=='a'){
        shipments_1.erase(shipments_1.begin());
        agv1_state=false;
      }
    }
    else{
      if(msg->data[0]=='p'){
        agv1_state=true;
      }
    }
  }

  void agv_2_callback(std_msgs::String::ConstPtr &msg){
    if(agv2_state){
      if(msg->data[0]=='r' && msg->data[2]=='a'){
        shipments_2.erase(shipments_2.begin());
        agv2_state=false;
      }
    }
    else{
      if(msg->data[0]=='p'){
        agv2_state=true;
      }
    }
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
    // else{
    //   ROS_INFO("Gripper closed");
    // }

  }
  bool faul_1=false;
  double faul_1_x=0;
  double faul_1_y=0;
  void quality_ctrl_1(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if(image_msg->models.size()>0){
      faul_1=true;
      auto& item = image_msg->models[0];
      faul_1_x = -item.pose.position.y;
      faul_1_y = max(item.pose.position.z-0.35, -0.24); // limitations
    }
    else{
      faul_1=false;
    }
  }
  bool faul_2=false;
  double faul_2_x=0;
  double faul_2_y=0;
  void quality_ctrl_2(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if(image_msg->models.size()>0){
      faul_2=true;
      auto& item = image_msg->models[0];
      faul_2_x = item.pose.position.y;
      faul_2_y = max(item.pose.position.z-0.35, -0.24);
    }
    else{
      faul_2=false;
    }
  }
private:
  std::string competition_state_;
  double current_score_;

  double belt_power;

  bool trans_1, trans_2;

  vector<double> arm_1_joint;
  vector<double> arm_2_joint;

  vector<double> arm_1_joint_goal, arm_2_joint_goal;

  double arm_1_linear, arm_2_linear;

  double arm_1_linear_goal, arm_2_linear_goal;

  const tf2::Quaternion q_logical=tf2::Quaternion(0.0, -0.70707272301, 0.0, 0.707140837031);

  deque<pair<ros::Time, PType>> events;
  // deque<ros::Time> event;

  const double maxBeltVel = 0.2;
  // 'bin1': [-0.3, -1.916, 0],
  // 'bin2': [-0.3, -1.15, 0],
  // 'bin3': [-0.3, -0.383, 0],
  // 'bin4': [-0.3, 0.383, 0],
  // 'bin5': [-0.3, 1.15, 0],
  // 'bin6': [-0.3, 1.916, 0],
  const double bin1_y = -1.916;
  const double bin2_y = -1.15;
  const double bin3_y = -0.383;
  const double bin4_y = 0.383;
  const double bin5_y = 1.15;
  const double bin6_y = 1.916;

  const vector<double> bin_y{-1.916, -1.15, -0.383, 0.383, 1.15, 1.916};
  const double tray_1_y = 3.15;
  const double tray_2_y = -3.15;

  const double arm_1_zero = 0.92;
  const double arm_2_zero = -0.92;

  const double logical_camera_x = 0.8;
  const double logical_camera_z = 1.15;

  const vector<double> classify_pos_1=invkinematic(vector<double>{-0.55, 0.75, 0.47});
  const vector<double> classify_pos_2=invkinematic(vector<double>{-0.55, -0.75, 0.47});
  
  const vector<double> classify2bpos_1=invkinematic(vector<double>{0.45, 0.45, 0.47});
  const vector<double> classify2bpos_2=invkinematic(vector<double>{0.45, -0.45, 0.47});

  const vector<double> throw_1_1 = invkinematic(vector<double>{0.3, -0.65, 0.2});
  const vector<double> throw_1_2 = invkinematic(vector<double>{0.3, -0.65, 0.1});
  
  const vector<double> throw_2_1 = invkinematic(vector<double>{0.3, 0.65, 0.2});
  const vector<double> throw_2_2 = invkinematic(vector<double>{0.3, 0.65, 0.1});

  const vector<double> desk_hand_1_0 = invkinematic(vector<double>{-0.00001, 0.92, 0.35});

  vector<double> desk_hand_1_1_pose {-0.00001, 0.92, 0.18};
  vector<double> desk_hand_1_2_pose {-0.00001, 0.92, 0.1};

  const vector<double> desk_hand_1_1 = invkinematic(vector<double>{-0.00001, 0.92, 0.18});
  const vector<double> desk_hand_1_2 = invkinematic(vector<double>{-0.00001, 0.92, 0.05});
  const vector<double> desk_hand_1_3 = invkinematic(vector<double>{-0.00001, 0.72, 0.30});
  const vector<double> desk_hand_1_4 = invkinematic(vector<double>{-0.00001, 0.92, -0.04});

  const vector<double> desk_hand_1_5 = invkinematic(vector<double>{-0.00001, 0.98, 0.1});
  const vector<double> desk_hand_1_6 = invkinematic(vector<double>{-0.00001, 0.98, -0.04});
  const vector<double> desk_hand_1_7 = invkinematic(vector<double>{-0.00001, 0.86, 0.1});
  const vector<double> desk_hand_1_8 = invkinematic(vector<double>{-0.00001, 0.86, -0.04});

  const vector<double> desk_hand_1_9=invkinematic(vector<double>{0.05, 0.92, 0.1});
  const vector<double> desk_hand_1_10=invkinematic(vector<double>{0.05, 0.92, -0.04});
  const vector<double> desk_hand_1_11=invkinematic(vector<double>{-0.05, 0.92, 0.1});
  const vector<double> desk_hand_1_12=invkinematic(vector<double>{-0.05, 0.92, -0.04});

  // const vector<double> desk_hand_1_13=invkinematic(vector<double>{-0.06, 0.88, 0.1});
  // const vector<double> desk_hand_1_14=invkinematic(vector<double>{-0.06, 0.88, -0.04});
  // const vector<double> desk_hand_1_15=invkinematic(vector<double>{0.02, 0.88, 0.1});
  // const vector<double> desk_hand_1_16=invkinematic(vector<double>{0.02, 0.88, -0.04});
  // const vector<double> desk_hand_1_17=invkinematic(vector<double>{-0.06, 0.91, 0.1});
  // const vector<double> desk_hand_1_18=invkinematic(vector<double>{-0.06, 0.91, -0.04});
  // const vector<double> desk_hand_1_19=invkinematic(vector<double>{0.2, 0.91, 0.1});
  // const vector<double> desk_hand_1_20=invkinematic(vector<double>{0.2, 0.91, -0.04});
  
  const vector<double> desk_hand_2_0=invkinematic(vector<double>{-0.00001, -0.92, 0.35});  

  vector<double> desk_hand_2_1_pose {-0.00001, -0.92, 0.18};
  vector<double> desk_hand_2_2_pose {-0.00001, -0.92, 0.1};

  const vector<double> desk_hand_2_1=invkinematic(vector<double>{-0.00001, -0.92, 0.18});
  const vector<double> desk_hand_2_2=invkinematic(vector<double>{-0.00001, -0.92, 0.05});
  const vector<double> desk_hand_2_3=invkinematic(vector<double>{-0.00001, -0.72, 0.30});
  const vector<double> desk_hand_2_4=invkinematic(vector<double>{-0.00001, -0.92, -0.04});

  const vector<double> desk_hand_2_5=invkinematic(vector<double>{-0.00001, -0.98, 0.1});
  const vector<double> desk_hand_2_6=invkinematic(vector<double>{-0.00001, -0.98, -0.04});
  const vector<double> desk_hand_2_7=invkinematic(vector<double>{-0.00001, -0.86, 0.1});
  const vector<double> desk_hand_2_8=invkinematic(vector<double>{-0.00001, -0.86, -0.04});
  
  const vector<double> desk_hand_2_9=invkinematic(vector<double>{0.05, -0.92, 0.1});
  const vector<double> desk_hand_2_10=invkinematic(vector<double>{0.05, -0.92, -0.04});
  const vector<double> desk_hand_2_11=invkinematic(vector<double>{-0.05, -0.92, 0.1});
  const vector<double> desk_hand_2_12=invkinematic(vector<double>{-0.05, -0.92, -0.04});
  

  vector<double> flip_pose_1 = invkinematic(vector<double>{-0.0265, 0.8, 0.47});
  vector<double> flip_pose_2 = invkinematic(vector<double>{0.0265, -0.8, 0.47});

  double x_r_1, y_r_1;
  double x_d_1, y_d_1;
  double x_r_2, y_r_2;
  double x_d_2, y_d_2;


  ros::Publisher arm_1_joint_trajectory_publisher_;
  ros::Publisher arm_2_joint_trajectory_publisher_;
  
  ros::ServiceClient arm_1_gripper_ctrl;
  ros::ServiceClient arm_2_gripper_ctrl;
  ros::ServiceClient agv_1;
  ros::ServiceClient agv_2;

  // std::vector<osrf_gear::Order> received_orders_;
  // vector<Order> received_orders_; //order ,shipment, each part
  sensor_msgs::JointState arm_1_current_joint_states_;
  sensor_msgs::JointState arm_2_current_joint_states_;
  bool arm_1_has_been_zeroed_=false;
  bool arm_2_has_been_zeroed_=false;

  State arm_1_state;
  State arm_2_state;

  bool catched_1=false, catched_2=false;
  int bin_num_1, bin_num_2; // 0: belt 1-6, bins

  bool enabled_1, enabled_2;

  bool fum_1_init, fum_2_init;

  bool agv1_state, agv2_state;

  const vector<double> rest_joints{0, -2, -1, 0, 0, 0};
  

  vector<int> bin_type;

  double dtheta_1, dtheta_2;

  bool flipok=false;
  int flip_lock=0;
  bool trans_lock=false; // too lazy to use real lock, mostly ok
  bool signal_1=false, signal_2=false;

};

// void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
//   if ((msg->max_range - msg->range) > 0.01) {  // If there is an object in proximity.
//     ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
//   }
// }



// %Tag(MAIN)%
int main(int argc, char ** argv) {
  // Last argument is the default name of the node.
  ros::init(argc, argv, "ariac_example_node");

  ros::NodeHandle node;

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);

  // Subscribe to the '/ariac/current_score' topic.
  // ros::Subscriber current_score_subscriber = node.subscribe(
  //   "/ariac/current_score", 10,
  //   &MyCompetitionClass::current_score_callback, &comp_class);

  // Subscribe to the '/ariac/competition_state' topic.
  // ros::Subscriber competition_state_subscriber = node.subscribe(
  //   "/ariac/competition_state", 10,
  //   &MyCompetitionClass::competition_state_callback, &comp_class);

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

  // Subscribe to the '/ariac/logical_camera_1' topic.
  ros::Subscriber logical_camera_subscriber = node.subscribe(
    "/ariac/logical_camera_0", 10,
    &MyCompetitionClass::logical_camera_callback, &comp_class);

  // Subscribe to the '/ariac/laser_profiler_1' topic.
  
  //TODO:DECISION

  ros::Subscriber laser_profiler_subscriber = node.subscribe(
    "/ariac/laser_profiler_1", 10, 
    &MyCompetitionClass::laser_profiler_callback, &comp_class);

  ros::Subscriber belt_state_subscriber = node.subscribe(
    "/ariac/conveyor/state", 10, 
    &MyCompetitionClass::belt_state_callback, &comp_class);
  
  ros::Subscriber gripper_1_state_subscriber = node.subscribe(
    "/ariac/arm1/gripper/state", 10, 
    &MyCompetitionClass::gripper_1_callback, &comp_class);

  ros::Subscriber gripper_2_state_subscriber = node.subscribe(
    "/ariac/arm2/gripper/state", 10, 
    &MyCompetitionClass::gripper_2_callback, &comp_class);


  //faulty

  ros::Subscriber quality_ctrl_1_subscriber = node.subscribe(
    "/ariac/quality_control_sensor_1", 10,
    &MyCompetitionClass::quality_ctrl_1, &comp_class);  

  ros::Subscriber quality_ctrl_2_subscriber = node.subscribe(
    "/ariac/quality_control_sensor_2", 10,
    &MyCompetitionClass::quality_ctrl_2, &comp_class);
  
  ROS_INFO("Setup complete.");
  start_competition(node);
  ros::spin();  // This executes callbacks on new data until ctrl-c.

  return 0;
}
// %EndTag(MAIN)%
// %EndTag(FULLTEXT)%
