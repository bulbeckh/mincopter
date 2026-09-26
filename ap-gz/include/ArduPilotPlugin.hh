/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef ARDUPILOTPLUGIN_HH_
#define ARDUPILOTPLUGIN_HH_

#include <array>
#include <memory>

#include <gz/sim/System.hh>
#include <sdf/sdf.hh>

#include "SimulationMessage.h"

namespace gz
{
namespace sim
{
namespace systems
{

// Forward declare private data class
class ArduPilotSocketPrivate;
class ArduPilotPluginPrivate;

/// \brief Interface ArduPilot from ardupilot stack
/// modeled after SITL/SIM_*
///
/// The plugin requires the following parameters:
/// <control>             control description block
///    <!-- inputs from Ardupilot -->
///    "channel"          attribute, ardupilot control channel
///    <multiplier>       command multiplier
///    <offset>           command offset
///    <servo_max>        upper limit for PWM input
///    <servo_min>        lower limit for PWM input
///    <!-- output to Gazebo -->
///    <type>             type of control, VELOCITY, POSITION, EFFORT or COMMAND
///    <useForce>         1 if joint forces are applied, 0 to set joint directly
///    <p_gain>           velocity pid p gain
///    <i_gain>           velocity pid i gain
///    <d_gain>           velocity pid d gain
///    <i_max>            velocity pid max integral correction
///    <i_min>            velocity pid min integral correction
///    <cmd_max>          velocity pid max command torque
///    <cmd_min>          velocity pid min command torque
///    <jointName>        motor joint, torque applied here
///    <cmd_topic>        topic to publish commands that are processed
///                       by other plugins
///
///    <turningDirection> rotor turning direction, 'cw' or 'ccw'
///    <frequencyCutoff>  filter incoming joint state
///    <samplingRate>     sampling rate for filtering incoming joint state
///    <rotorVelocitySlowdownSim> for rotor aliasing problem, experimental
///
/// <imuName>     scoped name for the imu sensor
/// <anemometer>  scoped name for the wind sensor
/// <connectionTimeoutMaxCount> timeout before giving up on
///                             controller synchronization
/// <have_32_channels>    set true if 32 channels are enabled
///
class GZ_SIM_VISIBLE ArduPilotPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{
  /// \brief Constructor.
  public: ArduPilotPlugin();

  /// \brief Destructor.
  public: ~ArduPilotPlugin();

  public: void Reset(const UpdateInfo &_info,
                      EntityComponentManager &_ecm) final;

  /// \brief Load configuration from SDF on startup.
  public: void Configure(const gz::sim::Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        gz::sim::EntityComponentManager &_ecm,
                        gz::sim::EventManager &_eventMgr) final;

  /// \brief Do the part of one update loop that involves making
  ///        changes to simulation.
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm) final;

  /// \brief Do the part of one update loop that involves
  ///        reading results from simulation.
  public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                          const gz::sim::EntityComponentManager &_ecm) final;

  /// \brief Load control channels
  private: void LoadControlChannels(
      sdf::ElementPtr _sdf,
      gz::sim::EntityComponentManager &_ecm);

  /// \brief Load IMU sensors
  private: void LoadImuSensors(
      sdf::ElementPtr _sdf,
      gz::sim::EntityComponentManager &_ecm);

  /// \brief Update the control surfaces controllers.
  /// \param[in] _info Update information provided by the server.
  private: void OnUpdate();

  /// \brief Update PID Joint controllers.
  /// \param[in] _dt time step size since last update.
  private: void ApplyMotorForces(
      const double _dt,
      gz::sim::EntityComponentManager &_ecm);

  /// \brief Reset PID Joint controllers.
  private: void ResetPIDs();

  private: bool ReceiveMessage();

  /// \brief Update the motor commands given servo PWM values
  private: void UpdateMotorCommands(const std::array<uint16_t, 4> &_pwm);

  /// \brief Create the state JSON
  private: void CreateStateJSON(
			   mc::StateMessage& _msg,
      double _simTime,
	  uint64_t _iterations,
      const gz::sim::EntityComponentManager &_ecm) const;

  /// \brief Send state to ArduPilot
  private: void SendState(mc::StateMessage& _msg) const;

  /// \brief Initialise flight dynamics model socket
  private: bool InitSockets(sdf::ElementPtr _sdf) const;

  private: gz::sim::EventManager* eventMgr{nullptr};

  /// \brief Private data pointer.
  private: std::unique_ptr<ArduPilotPluginPrivate> dataPtr;

};

}  // namespace systems
}  // namespace sim
}  // namespace gz

#endif  // ARDUPILOTPLUGIN_HH_
