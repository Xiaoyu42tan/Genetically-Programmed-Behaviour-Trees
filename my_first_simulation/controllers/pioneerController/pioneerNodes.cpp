#include "pioneerNodes.h"

using namespace BT;
using namespace webots;

#define MAX_SPEED 4

///////////////////

MoveForward::MoveForward(const std::string& name, const NodeConfig& config, Robot* robot): SyncActionNode(name, config), _robot(robot) {
    _leftMotor = _robot->getMotor("left wheel motor");
    _rightMotor = _robot->getMotor("right wheel motor");
}

PortsList MoveForward::providedPorts() { return {}; }

NodeStatus MoveForward::tick() {
    _leftMotor->setPosition(INFINITY);
    _rightMotor->setPosition(INFINITY);
    _leftMotor->setVelocity(MAX_SPEED);
    _rightMotor->setVelocity(MAX_SPEED);
    return NodeStatus::FAILURE;
}

///////////////////

SpinLeft::SpinLeft(const std::string& name, const NodeConfig& config, Robot* robot): SyncActionNode(name, config), _robot(robot) {
    _leftMotor = _robot->getMotor("left wheel motor");
    _rightMotor = _robot->getMotor("right wheel motor");
}

PortsList SpinLeft::providedPorts() { return {}; }

NodeStatus SpinLeft::tick() {
    _leftMotor->setPosition(INFINITY);
    _rightMotor->setPosition(INFINITY);
    _leftMotor->setVelocity(-MAX_SPEED);
    _rightMotor->setVelocity(MAX_SPEED);
    return NodeStatus::FAILURE;
}

///////////////////

SpinRight::SpinRight(const std::string& name, const NodeConfig& config, Robot* robot): SyncActionNode(name, config), _robot(robot) {
    _leftMotor = _robot->getMotor("left wheel motor");
    _rightMotor = _robot->getMotor("right wheel motor");
}

PortsList SpinRight::providedPorts() { return {}; }

NodeStatus SpinRight::tick() {
    _leftMotor->setPosition(INFINITY);
    _rightMotor->setPosition(INFINITY);
    _leftMotor->setVelocity(MAX_SPEED);
    _rightMotor->setVelocity(-MAX_SPEED);
    return NodeStatus::FAILURE;
}

///////////////////

MoveBackward::MoveBackward(const std::string& name, const NodeConfig& config, Robot* robot): SyncActionNode(name, config), _robot(robot) {
    _leftMotor = _robot->getMotor("left wheel motor");
    _rightMotor = _robot->getMotor("right wheel motor");
}

PortsList MoveBackward::providedPorts() { return {}; }

NodeStatus MoveBackward::tick() {
    _leftMotor->setPosition(INFINITY);
    _rightMotor->setPosition(INFINITY);
    _leftMotor->setVelocity(-MAX_SPEED);
    _rightMotor->setVelocity(-MAX_SPEED);
    return NodeStatus::FAILURE;
}

///////////////////

CheckDS0::CheckDS0(const std::string& name, const NodeConfig& config, Robot* robot): SyncActionNode(name, config), _robot(robot) {
    _sensor = _robot->getDistanceSensor("ds0");
    _sensor->enable(16);
}

PortsList CheckDS0::providedPorts() { return {}; }

NodeStatus CheckDS0::tick() {
    if (_sensor->getValue() > 0.1) {
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

///////////////////

CheckDS15::CheckDS15(const std::string& name, const NodeConfig& config, Robot* robot): SyncActionNode(name, config), _robot(robot) {
    _sensor = _robot->getDistanceSensor("ds15");
    _sensor->enable(16);
}

PortsList CheckDS15::providedPorts() { return {}; }

NodeStatus CheckDS15::tick() {
    if (_sensor->getValue() > 0.1) {
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

///////////////////

CheckDS13::CheckDS13(const std::string& name, const NodeConfig& config, Robot* robot): SyncActionNode(name, config), _robot(robot) {
    _sensor = _robot->getDistanceSensor("ds13");
    _sensor->enable(16);
}

PortsList CheckDS13::providedPorts() { return {}; }

NodeStatus CheckDS13::tick() {
    if (_sensor->getValue() > 0.1) {
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

///////////////////

CheckDS2::CheckDS2(const std::string& name, const NodeConfig& config, Robot* robot): SyncActionNode(name, config), _robot(robot) {
    _sensor = _robot->getDistanceSensor("ds2");
    _sensor->enable(16);
}

PortsList CheckDS2::providedPorts() { return {}; }

NodeStatus CheckDS2::tick() {
    if (_sensor->getValue() > 0.1) {
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

///////////////////

testForward::testForward(const std::string& name, const NodeConfig& config, Robot* robot): StatefulActionNode(name, config), _robot(robot) {
    _sensor = _robot->getDistanceSensor("ds0");
    _sensor->enable(16);

    _leftMotor = _robot->getMotor("left wheel motor");
    _rightMotor = _robot->getMotor("right wheel motor");
}

PortsList testForward::providedPorts() { return {}; }

NodeStatus testForward::onStart() {
    std::cout << "what\n";
    _leftMotor->setPosition(INFINITY);
    _rightMotor->setPosition(INFINITY);
    _leftMotor->setVelocity(MAX_SPEED);
    _rightMotor->setVelocity(MAX_SPEED);
    return NodeStatus::RUNNING;
}

NodeStatus testForward::onRunning() {
    if (_sensor->getValue() < 0.5) {
        hasRun = true;
        return NodeStatus::FAILURE;
    }
    return NodeStatus::RUNNING;
}

void testForward::onHalted() {
    _leftMotor->setVelocity(0);
    _rightMotor->setVelocity(0);
}



testSpin::testSpin(const std::string& name, const NodeConfig& config, Robot* robot): StatefulActionNode(name, config), _robot(robot) {
    _sensor = _robot->getDistanceSensor("ds0");
    _sensor->enable(16);

    _leftMotor = _robot->getMotor("left wheel motor");
    _rightMotor = _robot->getMotor("right wheel motor");
}

PortsList testSpin::providedPorts() { return {}; }

NodeStatus testSpin::onStart() {
    
    _leftMotor->setPosition(INFINITY);
    _rightMotor->setPosition(INFINITY);
    _leftMotor->setVelocity(-MAX_SPEED);
    _rightMotor->setVelocity(MAX_SPEED);
    return NodeStatus::RUNNING;
}

NodeStatus testSpin::onRunning() {
    counter++;
    if (counter > 1000) {
        _leftMotor->setVelocity(0);
        _rightMotor->setVelocity(0);
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
}

void testSpin::onHalted() {
    _leftMotor->setVelocity(0);
    _rightMotor->setVelocity(0);
}