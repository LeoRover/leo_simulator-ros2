#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components.hh>

namespace leo_gz
{

class DifferentialSystem
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemUpdate
{
  /// Model entity
  gz::sim::Model model_{gz::sim::kNullEntity};

  /// Force multiplier constant
  double force_constant_;

  /// Joint entities
  gz::sim::Entity joint_a_, joint_b_;

  /// Whether the system has been properly configured
  bool configured_{false};

public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & eventMgr) override
  {
    model_ = gz::sim::Model(entity);

    if (!model_.Valid(ecm)) {
      gzerr << "DifferentialSystem plugin should be attached to a model "
            << "entity. Failed to initialize." << std::endl;
      return;
    }

    if (!sdf->HasElement("jointA")) {
      gzerr << "No jointA element present. DifferentialSystem could not be loaded." << std::endl;
      return;
    }
    auto joint_a_name_ = sdf->Get<std::string>("jointA");

    if (!sdf->HasElement("jointB")) {
      gzerr << "No jointB element present. DifferentialSystem could not be loaded." << std::endl;
      return;
    }
    auto joint_b_name_ = sdf->Get<std::string>("jointB");

    if (!sdf->HasElement("forceConstant")) {
      gzerr << "No forceConstant element present. DifferentialSystem could not be loaded." <<
        std::endl;
      return;
    }
    force_constant_ = sdf->Get<double>("forceConstant");

    joint_a_ = model_.JointByName(ecm, joint_a_name_);
    if (joint_a_ == gz::sim::kNullEntity) {
      gzerr << "Failed to find joint named \'" << joint_a_name_ << "\'" << std::endl;
      return;
    }

    joint_b_ = model_.JointByName(ecm, joint_b_name_);
    if (joint_b_ == gz::sim::kNullEntity) {
      gzerr << "Failed to find joint named \'" << joint_b_name_ << "\'" << std::endl;
      return;
    }

    if (!ecm.EntityHasComponentType(joint_a_, gz::sim::components::JointPosition().TypeId())) {
      gzmsg << "Joint A does not have JointPosition component. Creating one..." << std::endl;
      ecm.CreateComponent(joint_a_, gz::sim::components::JointPosition());
    }

    if (!ecm.EntityHasComponentType(joint_b_, gz::sim::components::JointPosition().TypeId())) {
      gzmsg << "Joint B does not have JointPosition component. Creating one..." << std::endl;
      ecm.CreateComponent(joint_b_, gz::sim::components::JointPosition());
    }

    if (!ecm.EntityHasComponentType(joint_a_, gz::sim::components::JointForceCmd().TypeId())) {
      gzmsg << "Joint A does not have JointForceCmd component. Creating one..." << std::endl;
      ecm.CreateComponent(joint_a_, gz::sim::components::JointForceCmd({0}));
    }

    if (!ecm.EntityHasComponentType(joint_b_, gz::sim::components::JointForceCmd().TypeId())) {
      gzmsg << "Joint B does not have JointForceCmd component. Creating one..." << std::endl;
      ecm.CreateComponent(joint_b_, gz::sim::components::JointForceCmd({0}));
    }

    configured_ = true;
  }

  void Update(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!configured_ || info.paused) {return;}

    // Retrieve components
    auto pos_a_component = ecm.Component<gz::sim::components::JointPosition>(joint_a_);
    auto pos_b_component = ecm.Component<gz::sim::components::JointPosition>(joint_b_);
    auto force_cmd_a_component = ecm.Component<gz::sim::components::JointForceCmd>(joint_a_);
    auto force_cmd_b_component = ecm.Component<gz::sim::components::JointForceCmd>(joint_b_);

    double pos_a = pos_a_component->Data()[0];
    double pos_b = pos_b_component->Data()[0];
    double angle_diff = pos_a - pos_b;

    double current_cmd_a = force_cmd_a_component->Data()[0];
    double current_cmd_b = force_cmd_b_component->Data()[0];

    *force_cmd_a_component = gz::sim::components::JointForceCmd(
      {current_cmd_a - angle_diff * force_constant_});
    *force_cmd_b_component = gz::sim::components::JointForceCmd(
      {current_cmd_b + angle_diff * force_constant_});
  }
};

}

#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(
  leo_gz::DifferentialSystem,
  gz::sim::System,
  leo_gz::DifferentialSystem::ISystemConfigure,
  leo_gz::DifferentialSystem::ISystemUpdate)
