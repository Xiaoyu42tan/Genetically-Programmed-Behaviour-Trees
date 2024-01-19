#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <webots/Camera.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Supervisor.hpp>

#include <string>

using namespace BT;
using namespace webots;



class grabBox : public StatefulActionNode 
{
    private:
    Supervisor* _robot;

    Motor *base;
    Motor *forearm;
    Motor *left_gripper;
    Motor *right_gripper;
    Motor *rotational_wrist;
    Motor *upperarm;
    Motor *wrist;

    PositionSensor *base_sensor;
    PositionSensor *forearm_sensor;
    PositionSensor *left_gripper_sensor;
    PositionSensor *right_gripper_sensor;
    PositionSensor *rotational_wrist_sensor;
    PositionSensor *upperarm_sensor;
    PositionSensor *wrist_sensor;

    TouchSensor *ts1;
    TouchSensor *ts3;

    Camera *camera;

    int counter;
    int failCounter;
    int tsCounter;

    public:
    grabBox(const std::string& name, const NodeConfig& config, Supervisor* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;


};

class dropBox : public StatefulActionNode 
{
    private:
    Supervisor* _robot;

    Motor *base;
    Motor *forearm;
    Motor *left_gripper;
    Motor *right_gripper;
    Motor *rotational_wrist;
    Motor *upperarm;
    Motor *wrist;

    PositionSensor *base_sensor;
    PositionSensor *forearm_sensor;
    PositionSensor *left_gripper_sensor;
    PositionSensor *right_gripper_sensor;
    PositionSensor *rotational_wrist_sensor;
    PositionSensor *upperarm_sensor;
    PositionSensor *wrist_sensor;

    TouchSensor *ts1;
    TouchSensor *ts3;

    Camera *camera;

    int counter;

    bool wasSuccessful;

    public:
    dropBox(const std::string& name, const NodeConfig& config, Supervisor* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;


};

class findBox : public StatefulActionNode 
{
    private:
    Supervisor* _robot;

    Motor *base;
    Motor *forearm;
    Motor *left_gripper;
    Motor *right_gripper;
    Motor *rotational_wrist;
    Motor *upperarm;
    Motor *wrist;

    PositionSensor *base_sensor;
    PositionSensor *forearm_sensor;
    PositionSensor *left_gripper_sensor;
    PositionSensor *right_gripper_sensor;
    PositionSensor *rotational_wrist_sensor;
    PositionSensor *upperarm_sensor;
    PositionSensor *wrist_sensor;

    TouchSensor *ts1;
    TouchSensor *ts3;

    Camera *camera;

    int counter;

    public:
    findBox(const std::string& name, const NodeConfig& config, Supervisor* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;


};

class checkIfFinished : public StatefulActionNode 
{
    private:
    Supervisor* _robot;

    Motor *base;
    Motor *forearm;
    Motor *left_gripper;
    Motor *right_gripper;
    Motor *rotational_wrist;
    Motor *upperarm;
    Motor *wrist;

    PositionSensor *base_sensor;
    PositionSensor *forearm_sensor;
    PositionSensor *left_gripper_sensor;
    PositionSensor *right_gripper_sensor;
    PositionSensor *rotational_wrist_sensor;
    PositionSensor *upperarm_sensor;
    PositionSensor *wrist_sensor;

    TouchSensor *ts1;
    TouchSensor *ts3;

    Camera *camera;

    int counter;

    public:
    checkIfFinished(const std::string& name, const NodeConfig& config, Supervisor* robot);

    static PortsList providedPorts();

    // You must override the virtual function tick()
    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;


};
