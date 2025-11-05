#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class AeroPlugin : public ModelPlugin
  {
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the link we're applying force to
    private: physics::LinkPtr link;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // --- Configuration Parameters ---
    private: double drag_coeff = 1.4;  // Parachute Cd
    private: double area = 0.2;        // Parachute Area in m^2
    private: double rho = 1.225;       // Air density in kg/m^3 at sea level

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->link = this->model->GetLink("base_link"); // Get the link we defined in the URDF

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&AeroPlugin::OnUpdate, this));
      
      gzmsg << "Aerodynamics Plugin Loaded for model: " << this->model->GetName() << std::endl;
    }

    // Called by the world update start event
    public: void OnUpdate()
{
  // Get the current altitude (z position) of the CanSat
  double altitude = this->link->WorldPose().Pos().Z();

  // Only apply drag if the CanSat is below 500 meters (simulated parachute deployment)
  if (altitude < 500.0)
  {
    // Get the current linear velocity of the CanSat link
    ignition::math::Vector3d velocity = this->link->WorldLinearVel();

    if (velocity.Length() < 0.01)
    {
      return;
    }

    // Calculate the drag force: Fd = 0.5 * rho * v^2 * Cd * A
    double speed = velocity.Length();
    // Air density decreases with altitude, but we'll keep it simple
    double drag_magnitude = 0.5 * this->rho * speed * speed * this->drag_coeff * this->area;

    // The drag force is in the opposite direction of the velocity vector
    ignition::math::Vector3d drag_force = -velocity.Normalized() * drag_magnitude;

    // Apply the force to the link's center of mass
    this->link->AddForce(drag_force);
  }
}
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AeroPlugin)
}
