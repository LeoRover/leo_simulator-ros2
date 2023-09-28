#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components.hh>

namespace leo_gz
{

class DifferentialSystem
  : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemUpdate
{
  /// Model entity
  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};

  /// Force multiplier constant
  double force_constant_;

  /// Joint entities
  ignition::gazebo::Entity joint_a_, joint_b_;

  /// Whether the system has been properly configured
  bool configured_{false};

public:
  void Configure(
    const ignition::gazebo::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager & ecm,
    ignition::gazebo::EventManager & eventMgr) override
  {
    model_ = ignition::gazebo::Model(entity);

    if (!model_.Valid(ecm)) {
      ignerr << "DifferentialSystem plugin should be attached to a model "
            << "entity. Failed to initialize." << std::endl;
      return;
    }

    if (!sdf->HasElement("jointA")) {
      ignerr << "No jointA element present. DifferentialSystem could not be loaded." << std::endl;
      return;
    }
    auto joint_a_name_ = sdf->Get<std::string>("jointA");

    if (!sdf->HasElement("jointB")) {
      ignerr << "No jointB element present. DifferentialSystem could not be loaded." << std::endl;
      return;
    }
    auto joint_b_name_ = sdf->Get<std::string>("jointB");

    if (!sdf->HasElement("forceConstant")) {
      ignerr << "No forceConstant element present. DifferentialSystem could not be loaded." <<
        std::endl;
      return;
    }
    force_constant_ = sdf->Get<double>("forceConstant");

    joint_a_ = model_.JointByName(ecm, joint_a_name_);
    if (joint_a_ == ignition::gazebo::kNullEntity) {
      ignerr << "Failed to find joint named \'" << joint_a_name_ << "\'" << std::endl;
      return;
    }

    joint_b_ = model_.JointByName(ecm, joint_b_name_);
    if (joint_b_ == ignition::gazebo::kNullEntity) {
      ignerr << "Failed to find joint named \'" << joint_b_name_ << "\'" << std::endl;
      return;
    }

    if (!ecm.EntityHasComponentType(joint_a_, ignition::gazebo::components::JointPosition().TypeId())) {
      ignmsg << "Joint A does not have JointPosition component. Creating one..." << std::endl;
      ecm.CreateComponent(joint_a_, ignition::gazebo::components::JointPosition());
    }

    if (!ecm.EntityHasComponentType(joint_b_, ignition::gazebo::components::JointPosition().TypeId())) {
      ignmsg << "Joint B does not have JointPosition component. Creating one..." << std::endl;
      ecm.CreateComponent(joint_b_, ignition::gazebo::components::JointPosition());
    }

    if (!ecm.EntityHasComponentType(joint_a_, ignition::gazebo::components::JointForceCmd().TypeId())) {
      ignmsg << "Joint A does not have JointForceCmd component. Creating one..." << std::endl;
      ecm.CreateComponent(joint_a_, ignition::gazebo::components::JointForceCmd({0}));
    }

    if (!ecm.EntityHasComponentType(joint_b_, ignition::gazebo::components::JointForceCmd().TypeId())) {
      ignmsg << "Joint B does not have JointForceCmd component. Creating one..." << std::endl;
      ecm.CreateComponent(joint_b_, ignition::gazebo::components::JointForceCmd({0}));
    }

    configured_ = true;
  }

  void Update(
    const ignition::gazebo::UpdateInfo & info,
    ignition::gazebo::EntityComponentManager & ecm) override
  {
    if (!configured_ || info.paused) {return;}

    // Retrieve components
    auto pos_a_component = ecm.Component<ignition::gazebo::components::JointPosition>(joint_a_);
    auto pos_b_component = ecm.Component<ignition::gazebo::components::JointPosition>(joint_b_);
    auto force_cmd_a_component = ecm.Component<ignition::gazebo::components::JointForceCmd>(joint_a_);
    auto force_cmd_b_component = ecm.Component<ignition::gazebo::components::JointForceCmd>(joint_b_);

    double pos_a = pos_a_component->Data()[0];
    double pos_b = pos_b_component->Data()[0];
    double angle_diff = pos_a - pos_b;

    double current_cmd_a = force_cmd_a_component->Data()[0];
    double current_cmd_b = force_cmd_b_component->Data()[0];

    *force_cmd_a_component = ignition::gazebo::components::JointForceCmd(
      {current_cmd_a - angle_diff * force_constant_});
    *force_cmd_b_component = ignition::gazebo::components::JointForceCmd(
      {current_cmd_b + angle_diff * force_constant_});
  }
};

}

#include <gz/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
  leo_gz::DifferentialSystem,
  ignition::gazebo::System,
  leo_gz::DifferentialSystem::ISystemConfigure,
  leo_gz::DifferentialSystem::ISystemUpdate)
