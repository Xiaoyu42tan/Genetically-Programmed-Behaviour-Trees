#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <webots/Camera.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>

#include <string>

using namespace BT;
using namespace webots;

class MoveForward : public SyncActionNode 
{
    private:
    Robot* _robot;
    Motor* _leftMotor;
    Motor* _rightMotor;
    
    public:
    MoveForward(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus tick() override;
};

class SpinLeft : public SyncActionNode 
{
    private:
    Robot* _robot;
    Motor* _leftMotor;
    Motor* _rightMotor;
    
    public:
    SpinLeft(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus tick() override;

};

class SpinRight : public SyncActionNode 
{
    private:
    Robot* _robot;
    Motor* _leftMotor;
    Motor* _rightMotor;
    
    public:
    SpinRight(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus tick() override;

};

class MoveBackward : public SyncActionNode 
{
    private:
    Robot* _robot;
    Motor* _leftMotor;
    Motor* _rightMotor;
    
    public:
    MoveBackward(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus tick() override;
};

// front left
class CheckDS0 : public SyncActionNode 
{
    private:
    Robot* _robot;
    DistanceSensor* _sensor;
    
    public:
    CheckDS0(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus tick() override;
};

// front right
class CheckDS15 : public SyncActionNode 
{
    private:
    Robot* _robot;
    DistanceSensor* _sensor;
    
    public:
    CheckDS15(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus tick() override;
};

// side right
class CheckDS13 : public SyncActionNode 
{
    private:
    Robot* _robot;
    DistanceSensor* _sensor;
    
    public:
    CheckDS13(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus tick() override;
};

// side left
class CheckDS2 : public SyncActionNode 
{
    private:
    Robot* _robot;
    DistanceSensor* _sensor;
    
    public:
    CheckDS2(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus tick() override;
};

// testing
class testForward : public StatefulActionNode 
{
    private:
    Robot* _robot;
    DistanceSensor* _sensor;
    Motor* _leftMotor;
    Motor* _rightMotor;
    bool hasRun = false;
    
    public:
    testForward(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;


};

class testSpin : public StatefulActionNode 
{
    private:
    Robot* _robot;
    DistanceSensor* _sensor;
    Motor* _leftMotor;
    Motor* _rightMotor;
    int counter = 0;

    public:
    testSpin(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;


};