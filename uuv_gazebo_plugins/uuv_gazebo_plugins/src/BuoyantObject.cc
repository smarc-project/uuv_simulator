// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
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

#include <cmath>

#include <gazebo/gazebo.hh>
#include <gazebo/math/Box.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Vector3.hh>

#include "uuv_gazebo_plugins/BuoyantObject.hh"

namespace gazebo {

/////////////////////////////////////////////////
BuoyantObject::BuoyantObject(physics::LinkPtr _link)
{
  GZ_ASSERT(_link != NULL, "Invalid link pointer");

  // Initial volume
  this->volume = 0.0;
  // Fluid density for sea water at 0 degrees Celsius
  this->fluidDensity = 1028.0;
  this->g = 9.81;
  this->centerOfBuoyancy.Set(0, 0, 0);
  this->debugFlag = false;
  this->isSubmerged = true;
  this->metacentricWidth = 0.0;
  this->metacentricLength = 0.0;
  this->waterLevelPlaneArea = 0.0;
  this->submergedHeight = 0.0;
  this->isSurfaceVessel = false;

  // Buoyancy and orientation controls
  this->VBS = 0.0;
  this->LCG = 0.0;
  this->LCG_pitch_d_max = 0.06;
  this->LCG_pitch_mass = 2.6;
  this->VBS_capacity = 0.3;
  this->TCG_radius = 0.0294;
  this->TCG_mass = 0.22;

  this->link = _link;
  // Retrieve the bounding box
  // FIXME(mam0box) Gazebo's bounding box method is NOT working

  // TODO(mam0box) Change the way the bounding box is retrieved,
  // it should come from the physics engine but it is still not resolved
  this->boundingBox = link->GetBoundingBox();

  // Estimate volume, can be overwritten later
  this->SetVolume(-1);
  // Estimate CoB, can be overwritten later
  this->EstimateCoB();

  // Set neutrally buoyant flag to false
  this->neutrallyBuoyant = false;

  // ROS interface
  if (!ros::isInitialized()){
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "buoyancy_control",ros::init_options::NoSigintHandler);
  }

  this->rosNode.reset(new ros::NodeHandle("buoyancy_control"));

  ros::SubscribeOptions subs_pitch =
    ros::SubscribeOptions::create<std_msgs::Float64>(
        "/lcg_control_action",
        1,
        boost::bind(&BuoyantObject::pitchCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);
  this->subsPitch = this->rosNode->subscribe(subs_pitch);

  ros::SubscribeOptions subs_roll_1 =
    ros::SubscribeOptions::create<std_msgs::Float64>(
        "/tcg_control_action_1",
        1,
        boost::bind(&BuoyantObject::rollCB1, this, _1),
        ros::VoidPtr(), &this->rosQueue);
  this->subsRoll1 = this->rosNode->subscribe(subs_roll_1);

  ros::SubscribeOptions subs_roll_2 =
    ros::SubscribeOptions::create<std_msgs::Float64>(
        "/tcg_control_action_2",
        1,
        boost::bind(&BuoyantObject::rollCB2, this, _1),
        ros::VoidPtr(), &this->rosQueue);
  this->subsRoll2 = this->rosNode->subscribe(subs_roll_2);

  ros::SubscribeOptions subs_buoy =
    ros::SubscribeOptions::create<std_msgs::Float64>(
        "/vbs_control_action",
        1,
        boost::bind(&BuoyantObject::buoyancyCB, this, _1),
        ros::VoidPtr(), &this->rosQueue);
  this->subsDepth = this->rosNode->subscribe(subs_buoy);

  this->rosQueueThread = std::thread(std::bind(&BuoyantObject::QueueThread, this));
}

/////////////////////////////////////////////////
BuoyantObject::~BuoyantObject() {}

/////////////////////////////////////////////////
void BuoyantObject::pitchCB(const std_msgs::Float64ConstPtr &_msg){
    /// Pitch control input \in [0,100]:
    /// input maps to the distance the LCG mass is moved wrt the CoM of the vehicle to produce a torque
    /// input = 50 == no distance
    /// input = 0 == d_max/2
    /// input = 100 == - d_max/2

    const math::Pose pose = this->link->GetWorldPose();
    double distance_mass = -1 * (_msg->data - 50) * this->LCG_pitch_d_max / 100;
    this->LCG = distance_mass * cos(pose.rot.GetAsEuler().y) * this->LCG_pitch_mass * this->g;
}

/////////////////////////////////////////////////
void BuoyantObject::rollCB1(const std_msgs::Float64ConstPtr &_msg){

    /// Roll control, input 1 \in [-pi, pi]
    const math::Pose pose = this->link->GetWorldPose();
    this->TCG.x = (sin(_msg->data + pose.rot.GetAsEuler().x) *
                   this->TCG_radius * this->TCG_mass * this->g);
}

/////////////////////////////////////////////////
void BuoyantObject::rollCB2(const std_msgs::Float64ConstPtr &_msg){

    /// Roll control, input 2 \in [-pi, pi]
    const math::Pose pose = this->link->GetWorldPose();
    this->TCG.y = (sin(_msg->data + pose.rot.GetAsEuler().x) *
                   this->TCG_radius * this->TCG_mass * this->g);
}

/////////////////////////////////////////////////
void BuoyantObject::buoyancyCB(const std_msgs::Float64ConstPtr &_msg){
    /// Buoyancy control input \in [0,100]
    /// VBS_capacity is the volume of the VBS in SAM (0,3l)

    this->VBS = (_msg->data * this->VBS_capacity * this->fluidDensity) / 100;
}

/////////////////////////////////////////////////
void BuoyantObject::QueueThread(){
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}


/////////////////////////////////////////////////
void BuoyantObject::SetNeutrallyBuoyant()
{
  this->neutrallyBuoyant = true;
  // Calculate the equivalent volume for the submerged body
  // so that it will be neutrally buoyant
  this->volume = this->link->GetInertial()->GetMass() / this->fluidDensity;
  gzmsg << this->link->GetName() << " is neutrally buoyant" << std::endl;
}

/////////////////////////////////////////////////
void BuoyantObject::GetBuoyancyForce(const math::Pose &_pose,
  math::Vector3 &buoyancyForce, math::Vector3 &buoyancyTorque)
{
  double height = this->boundingBox.GetZLength();
  double z = _pose.pos.z;
  double volume = 0.0;

  buoyancyForce = math::Vector3(0, 0, 0);
  buoyancyTorque = math::Vector3(0, 0, 0);

  if (!this->isSurfaceVessel)
  {
    if (z + height / 2 > 0 && z < 0)
    {
      this->isSubmerged = false;
      volume = this->volume * (std::fabs(z) + height / 2) / height;
    }
    else if (z + height / 2 < 0)
    {
      this->isSubmerged = true;
      volume = this->volume;
    }

    if (!this->neutrallyBuoyant || volume != this->volume){
        buoyancyForce = math::Vector3(0, 0,
            (volume * this->fluidDensity - this->VBS) * this->g);
        buoyancyTorque = math::Vector3(this->TCG.x + this->TCG.y, this->LCG, 0);
    }
    else if (this->neutrallyBuoyant)
        buoyancyForce = math::Vector3(
            0, 0, this->link->GetInertial()->GetMass() * this->g);
  }
  else
  {
    // Implementation of the linear (small angle) theory for boxed-shaped
    // vessels. Further details can be seen at
    // T. I. Fossen, “Handbook of Marine Craft Hydrodynamics and Motion Control,” Apr. 2011.
    // Page 65
    if (this->waterLevelPlaneArea <= 0)
    {
      this->waterLevelPlaneArea = this->boundingBox.GetXLength() *
        this->boundingBox.GetYLength();
      gzmsg << this->link->GetName() << "::" << "waterLevelPlaneArea = " <<
        this->waterLevelPlaneArea << std::endl;
    }

    GZ_ASSERT(this->waterLevelPlaneArea > 0.0,
      "Water level plane area must be greater than zero");

    if (z + height / 2.0 > 0.5)
    {
      // Vessel is completely out of the water
      buoyancyForce = math::Vector3(0, 0, 0);
      buoyancyTorque = math::Vector3(0, 0, 0);
      // Store the restoring force vector, if needed
      this->StoreVector(RESTORING_FORCE, buoyancyForce);
      return;
    }
    else if (z + height / 2.0 < 0)
      this->submergedHeight = this->boundingBox.GetZLength();
    else
      this->submergedHeight = this->boundingBox.GetZLength() / 2 - z;

    volume = this->submergedHeight * this->waterLevelPlaneArea;
    buoyancyForce = math::Vector3(0, 0, volume * this->fluidDensity * this->g);
    buoyancyTorque = math::Vector3(
      -1 * this->metacentricWidth * sin(_pose.rot.GetAsEuler().x) * buoyancyForce.z,
      -1 * this->metacentricLength * sin(_pose.rot.GetAsEuler().y) * buoyancyForce.z,
      0);

  }

  // Store the restoring force vector, if needed
  this->StoreVector(RESTORING_FORCE, buoyancyForce);
}

/////////////////////////////////////////////////
void BuoyantObject::ApplyBuoyancyForce()
{
  // Link's pose
  const math::Pose pose = this->link->GetWorldPose();
  // Get the buoyancy force in world coordinates
  math::Vector3 buoyancyForce, buoyancyTorque;

  this->GetBuoyancyForce(pose, buoyancyForce, buoyancyTorque);

  GZ_ASSERT(!std::isnan(buoyancyForce.GetLength()),
    "Buoyancy force is invalid");
  GZ_ASSERT(!std::isnan(buoyancyTorque.GetLength()),
    "Buoyancy torque is invalid");

  if (!this->isSurfaceVessel){
      this->link->AddForceAtRelativePosition(buoyancyForce, this->GetCoB());
      this->link->AddTorque(buoyancyTorque);
  }
  else
  {
    this->link->AddRelativeForce(buoyancyForce);
    this->link->AddRelativeTorque(buoyancyTorque);
  }

}

/////////////////////////////////////////////////
void BuoyantObject::SetBoundingBox(const math::Box &_bBox)
{
  this->boundingBox = math::Box(_bBox);

  gzmsg << "New bounding box for " << this->link->GetName() << "::"
    << this->boundingBox << std::endl;
}

/////////////////////////////////////////////////
void BuoyantObject::SetVolume(double _volume)
{
  if (_volume > 0)
    this->volume = _volume;
  else
  {
    // If volume is not given, it will be estimated from the collision
    // geometries
    double v = 0.0;
    for (auto collision : link->GetCollisions())
      v += collision->GetShape()->ComputeVolume();

    this->volume = v;
  }
}

/////////////////////////////////////////////////
double BuoyantObject::GetVolume() { return this->volume; }

/////////////////////////////////////////////////
void BuoyantObject::EstimateCoB()
{
  // User did not provide center of buoyancy,
  // compute it from collision volume.
  double volumeSum = 0.0;
  math::Vector3 weightedPosSum = math::Vector3::Zero;
  for (auto collision : this->link->GetCollisions())
  {
    double volume = collision->GetShape()->ComputeVolume();
    volumeSum += volume;
    weightedPosSum += volume * collision->GetWorldPose().pos;
  }

  this->SetCoB(this->link->GetWorldPose().GetInverse().CoordPositionAdd(
      weightedPosSum / volumeSum));
}

/////////////////////////////////////////////////
void BuoyantObject::SetFluidDensity(double _fluidDensity)
{
  GZ_ASSERT(_fluidDensity > 0, "Fluid density must be a positive value");
  this->fluidDensity = _fluidDensity;
}

/////////////////////////////////////////////////
double BuoyantObject::GetFluidDensity() { return this->fluidDensity; }

/////////////////////////////////////////////////
void BuoyantObject::SetCoB(const math::Vector3 &_centerOfBuoyancy)
{
  this->centerOfBuoyancy.Set(_centerOfBuoyancy.x, _centerOfBuoyancy.y,
                             _centerOfBuoyancy.z);
}

/////////////////////////////////////////////////
math::Vector3 BuoyantObject::GetCoB() { return this->centerOfBuoyancy; }

/////////////////////////////////////////////////
void BuoyantObject::SetGravity(double _g)
{
  GZ_ASSERT(_g > 0, "Acceleration of gravity must be positive");
  this->g = _g;
}

/////////////////////////////////////////////////
double BuoyantObject::GetGravity() { return this->g; }

/////////////////////////////////////////////////
void BuoyantObject::SetDebugFlag(bool _debugOn) { this->debugFlag = _debugOn; }

/////////////////////////////////////////////////
bool BuoyantObject::GetDebugFlag() { return this->debugFlag; }

/////////////////////////////////////////////////
void BuoyantObject::SetStoreVector(std::string _tag)
{
  if (!this->debugFlag)
    return;
  // Test if field exists
  if (!this->hydroWrench.count(_tag))
    this->hydroWrench[_tag] = math::Vector3(0, 0, 0);
}

/////////////////////////////////////////////////
math::Vector3 BuoyantObject::GetStoredVector(std::string _tag)
{
  if (!this->debugFlag)
    return math::Vector3(0, 0, 0);
  if (this->hydroWrench.count(_tag))
    return this->hydroWrench[_tag];
  else
    return math::Vector3(0, 0, 0);
}

/////////////////////////////////////////////////
void BuoyantObject::StoreVector(std::string _tag, math::Vector3 _vec)
{
  if (!this->debugFlag)
    return;
  // Test if field exists
  if (this->hydroWrench.count(_tag))
    this->hydroWrench[_tag] = _vec;
}

/////////////////////////////////////////////////
bool BuoyantObject::IsSubmerged()
{
  return this->isSubmerged;
}

/////////////////////////////////////////////////
bool BuoyantObject::IsNeutrallyBuoyant()
{
  return this->neutrallyBuoyant;
}
}
