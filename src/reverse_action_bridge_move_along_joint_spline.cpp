// Copyright 2019 Fraunhofer IPA
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

#include <reverse_action_bridge/reverse_action_bridge.hpp>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <iiwa_msgs/MoveAlongJointSplineAction.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include <iiwa_msgs/action/move_along_joint_spline.hpp>

template<typename T1, typename T2>
static void copy_header_information(const T2 & header2, T1 & header1)
{
  header1.frame_id = header2.frame_id;
  header1.stamp.sec = header2.stamp.sec;
  header1.stamp.nsec = header2.stamp.nanosec;
}

template<typename T1, typename T2>
static void copy_joint_position(const T2 & joint2, T1 & joint1)
{
  copy_header_information(joint2.header, joint1.header);
  joint1.position.a1 = joint2.position.a1;
  joint1.position.a2 = joint2.position.a2;
  joint1.position.a3 = joint2.position.a3;
  joint1.position.a4 = joint2.position.a4;
  joint1.position.a5 = joint2.position.a5;
  joint1.position.a6 = joint2.position.a6;
  joint1.position.a7 = joint2.position.a7;
}

template<typename T1, typename T2>
static void copy_joint_spline(const T2 & spline2, T1 & spline1)
{
  const size_t spline_size = spline2.segments.size();
  spline1.segments.resize(spline_size);
  for(size_t i = 0; i < spline_size; ++i) {
    copy_joint_position(spline2.segments[i], spline1.segments[i]);
  }
}

using MoveAlongJointSplineActionBridge = ActionBridge<iiwa_msgs::MoveAlongJointSplineAction,
    iiwa_msgs::action::MoveAlongJointSpline>;

template<>
void MoveAlongJointSplineActionBridge::translate_goal_2_to_1(
  const ROS2Goal & goal2,
  ROS1Goal & goal1)
{
  copy_joint_spline(goal2.spline, goal1.spline);
}

template<>
void MoveAlongJointSplineActionBridge::translate_result_1_to_2(
  ROS2Result & result2,
  const ROS1Result & result1)
{
  result2.success = result1.success;
  result2.error = result1.error;
}

template<>
void MoveAlongJointSplineActionBridge::translate_feedback_1_to_2(
  ROS2Feedback & feedback2,
  const ROS1Feedback & feedback1)
{
  ;
}

int main(int argc, char * argv[])
{
  return MoveAlongJointSplineActionBridge::main("iiwa/action/move_along_joint_spline", argc, argv);
}
