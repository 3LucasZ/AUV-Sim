#include <ignition/msgs/double.pb.h>

#include <string>
#include <unordered_map>
#include <utility>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/sensors/Noise.hh>
#include <ignition/sensors/SensorFactory.hh>

#include <sdf/Sensor.hh>

#include <ignition/gazebo/components/CustomSensor.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>

#include "X150.hh"
#include "X150System.hh"

using namespace custom;

//////////////////////////////////////////////////
void X150System::PreUpdate(const ignition::gazebo::UpdateInfo &, ignition::gazebo::EntityComponentManager &_ecm) {
  _ecm.EachNew<ignition::gazebo::components::CustomSensor,
               ignition::gazebo::components::ParentEntity>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::CustomSensor *_custom,
        const ignition::gazebo::components::ParentEntity *_parent)->bool
      {
        // Get sensor's scoped name without the world
        auto sensorScopedName = ignition::gazebo::removeParentScope(ignition::gazebo::scopedName(_entity, _ecm, "::", false), "::");
        sdf::Sensor data = _custom->Data();
        data.SetName(sensorScopedName);

        // Default to scoped name as topic
        if (data.Topic().empty()) {
          std::string topic = scopedName(_entity, _ecm) + "/X150";
          data.SetTopic(topic);
        }

        // Default to 1 as update_rate (useless)
        if (data.UpdateRate()<0.001){
          data.SetUpdateRate(1.0);
        }

        ignition::sensors::SensorFactory sensorFactory;
        auto sensor = sensorFactory.CreateSensor<custom::X150>(data);
        if (nullptr == sensor){
          ignerr << "Failed to create X150 [" << sensorScopedName << "]" << std::endl;
          return false;
        }

        // Set sensor parent
        auto parentName = _ecm.Component<ignition::gazebo::components::Name>(_parent->Data())->Data();
        sensor->SetParent(parentName);

        // Set topic on Gazebo
        _ecm.CreateComponent(_entity, ignition::gazebo::components::SensorTopic(sensor->Topic()));

        // Keep track of this sensor
        this->entitySensorMap.insert(std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////***
void X150System::PostUpdate(const ignition::gazebo::UpdateInfo &_info, const ignition::gazebo::EntityComponentManager &_ecm) {
  // Only update and publish if not paused.
  if (!_info.paused) {
    for (auto &[entity, sensor] : this->entitySensorMap) {
      sensor->SetPose(ignition::gazebo::worldPose(entity, _ecm));
      sensor->Update(_info.simTime); 
    }
  }
  this->RemoveSensorEntities(_ecm);
}

//////////////////////////////////////////////////
void X150System::RemoveSensorEntities(const ignition::gazebo::EntityComponentManager &_ecm) {
  _ecm.EachRemoved<ignition::gazebo::components::CustomSensor>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::CustomSensor *)->bool
      {
        if (this->entitySensorMap.erase(_entity) == 0) {
          ignerr << "Internal error, missing X150 for entity [" << _entity << "]" << std::endl;
        }
        return true;
      });
}

IGNITION_ADD_PLUGIN(X150System, ignition::gazebo::System, X150System::ISystemPreUpdate, X150System::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(X150System, "custom::X150System")