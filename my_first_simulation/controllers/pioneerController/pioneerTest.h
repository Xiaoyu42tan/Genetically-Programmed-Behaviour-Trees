#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <webots/Camera.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include <string>

using namespace BT;
using namespace webots;

class MoveForward : public StatefulActionNode 
{
    private:
    Robot* _robot;
    Motor* _leftMotor;
    Motor* _rightMotor;
    PositionSensor* _leftSensor;
    PositionSensor* _rightSensor;
    bool _isMoving;

    double _prevLeftPosition;
    double _prevRightPosition;

    public:
    MoveForward(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // this function is invoked once at the beginning.
    NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

};

class MoveForwardConditional : public SyncActionNode 
{
    private:
    Robot* _robot;
    Motor* _leftMotor;
    Motor* _rightMotor;
    Camera* _camera;

    public:
    MoveForwardConditional(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus tick() override;

};

class CheckIfVisible : public SyncActionNode {
    private:
    Robot* _robot;
    Camera* _camera;

    public:
    CheckIfVisible(const std::string& name, const NodeConfig& config, Robot* _robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus tick() override;
};

class SpinLeft : public StatefulActionNode 
{
    private:
    Robot* _robot;
    Motor* _leftMotor;
    Motor* _rightMotor;
    Camera* _camera;

    public:
    SpinLeft(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // this function is invoked once at the beginning.
    NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

};

class SpinLeftConditional : public SyncActionNode 
{
    private:
    Robot* _robot;
    Motor* _leftMotor;
    Motor* _rightMotor;
    Camera* _camera;

    public:
    SpinLeftConditional(const std::string& name, const NodeConfig& config, Robot* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus tick() override;

};