#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/Keyboard.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <iostream>
using namespace BT;



BT::NodeStatus CheckBattery();

class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name);

  // You must override the virtual function tick()
  BT::NodeStatus tick() override;
};



// We want to wrap into an ActionNode the methods open() and close()
class GripperInterface
{
public:
  GripperInterface();
    
  NodeStatus open();

  NodeStatus close();

private:
  bool _open; // shared information
};
