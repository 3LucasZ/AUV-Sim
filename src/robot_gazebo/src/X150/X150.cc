/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <math.h>

#include <ignition/msgs/double.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/sensors/Noise.hh>
#include <ignition/sensors/Util.hh>

#include "X150.hh"

using namespace custom;

//////////////////////////////////////////////////
bool X150::Load(const sdf::Sensor &_sdf) {
  // Only load type X150
  auto type = ignition::sensors::customType(_sdf);
  if ("X150" != type) {
    ignerr << "Trying to load [X150] sensor, but got type [" << type << "] instead." << std::endl;
    return false;
  } else {
    ignerr << "Successfully loaded type [" << type << "]" << std::endl;
  }
  // Load common sensor params
  ignition::sensors::Sensor::Load(_sdf);
  // Advertise topic where data will be published
  this->pub = this->node.Advertise<ignition::msgs::Double>(this->Topic());
  if (!_sdf.Element()->HasElement("ignition:X150")) {
    ignerr << "No custom configuration for [" << this->Topic() << "]" << std::endl;
    return true;
  }
  // Load custom sensor params
  auto customElem = _sdf.Element()->GetElement("ignition:X150");
  // Load noise
  if (!customElem->HasElement("noise")){
    ignerr << "No noise for [" << this->Topic() << "] (fatal)" << std::endl;
    return false;
  } else {
    sdf::Noise noiseSdf;
    noiseSdf.Load(customElem->GetElement("noise"));
    this->noise = ignition::sensors::NoiseFactory::NewNoiseModel(noiseSdf);
    if (nullptr == this->noise) {
        ignerr << "Failed to load noise. (fatal)" << std::endl;
        return false;
    }
  }
  // Load X110 pose
  if (!customElem->HasElement("X110pose")){
    ignerr << "No X110 pose for [" << this->Topic() << "] (ok)" << std::endl;
  } else {
    auto customX110Pose = customElem->GetElement("X110pose");
    auto customX110PoseCfg = customElem->GetElement("X110pose")->GetValue()->GetAsString();
    std::istringstream iss(customX110PoseCfg);
    std::vector<double> split(6); 
    for (int i=0;i<6;i++) {
      iss >> split[i];
      if (iss.fail()) {
        ignerr << "Failed to load X110 pose (non double element). (fatal)" << std::endl;
        return false;
      }
    }
    if (!iss.eof()) {
        ignerr << "Failed to load X110 pose (too many elements). (fatal)" << std::endl;
        return false;
    }
    ignerr << customX110PoseCfg << std::endl;
  }
  return true;
}

//////////////////////////////////////////////////
bool X150::Update(const std::chrono::steady_clock::duration &_now) {
  ignition::msgs::Pose msg;
  *msg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  // this->curPose.Pos->X = this->noise->Apply(this->totalDistance);
  // msg.set_data(this->totalDistance);

  this->AddSequence(msg.mutable_header());
  this->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
void X150::SetPose(const ignition::math::Pose3d &_pose) {
  this->curPose = _pose;
}
