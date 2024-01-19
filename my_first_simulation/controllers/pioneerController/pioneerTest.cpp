#include "pioneerTest.h"

using namespace BT;
using namespace webots;

#define MAX_SPEED 4

////////////////////

MoveForward::MoveForward(const std::string& name, const NodeConfig& config, Robot* robot): StatefulActionNode(name, config), _robot(robot) {
    _leftMotor = _robot->getMotor("left wheel motor");
    _rightMotor = _robot->getMotor("right wheel motor");
    _leftSensor = robot->getPositionSensor("left wheel sensor");
    _rightSensor = robot->getPositionSensor("left wheel sensor");

    _leftSensor->enable(16);
    _rightSensor->enable(16);

}

PortsList MoveForward::providedPorts() { return {InputPort<double>("distance")}; }

NodeStatus MoveForward::onStart() {
    Expected<double> distance = getInput<double>("distance");

    if (!distance) {
        throw BT::RuntimeError("missing required input [distance]: ",distance.error());
    }
    
    _leftMotor->setPosition(distance.value());
    _rightMotor->setPosition(distance.value());
    _leftMotor->setVelocity(MAX_SPEED);
    _rightMotor->setVelocity(MAX_SPEED);

    _prevLeftPosition = _leftSensor->getValue();
    _prevRightPosition = _rightSensor->getValue();

    _isMoving = false;
    return NodeStatus::RUNNING;
}

NodeStatus MoveForward::onRunning() {
    double leftPosition = _leftSensor->getValue();
    double rightPosition = _rightSensor->getValue();

    double leftVelocity = (leftPosition - _prevLeftPosition) / 0.016;
    double rightVelocity = (rightPosition - _prevRightPosition) / 0.016;

    if (leftVelocity > 0.1 && rightVelocity > 0.1) {
        _isMoving = true;
    }

    if (leftVelocity == 0.0 && rightVelocity == 0.0 && _isMoving) {
        return NodeStatus::SUCCESS;
    }

    _prevLeftPosition = leftPosition;
    _prevRightPosition = rightPosition;

    return NodeStatus::RUNNING;
}

void MoveForward::onHalted() {
    _leftMotor->setVelocity(0);
    _rightMotor->setVelocity(0);
}

////////////////////

MoveForwardConditional::MoveForwardConditional(const std::string& name, const NodeConfig& config, Robot* robot): SyncActionNode(name, config), _robot(robot) {
    _leftMotor = _robot->getMotor("left wheel motor");
    _rightMotor = _robot->getMotor("right wheel motor");
    _camera = _robot->getCamera("pioneer_camera");
    _camera->recognitionEnable(50);
}

PortsList MoveForwardConditional::providedPorts() { return {}; }

NodeStatus MoveForwardConditional::tick() {
    _leftMotor->setPosition(INFINITY);
    _rightMotor->setPosition(INFINITY);
    _leftMotor->setVelocity(MAX_SPEED);
    _rightMotor->setVelocity(MAX_SPEED);
    return NodeStatus::FAILURE;
}

/////////////////

SpinLeft::SpinLeft(const std::string& name, const NodeConfig& config, Robot* robot): StatefulActionNode(name, config), _robot(robot) {
    _leftMotor = _robot->getMotor("left wheel motor");
    _rightMotor = _robot->getMotor("right wheel motor");
}

PortsList SpinLeft::providedPorts() { return {}; }

NodeStatus SpinLeft::onStart() {
    _leftMotor->setPosition(INFINITY);
    _rightMotor->setPosition(INFINITY);
    _leftMotor->setVelocity(-MAX_SPEED);
    _rightMotor->setVelocity(MAX_SPEED);
    return NodeStatus::RUNNING;
}

NodeStatus SpinLeft::onRunning() {
    return NodeStatus::RUNNING;
}

void SpinLeft::onHalted() {
    _leftMotor->setVelocity(0);
    _rightMotor->setVelocity(0);
    std::cout << "SpinLeft ABORTED\n";
}

/////////////////

SpinLeftConditional::SpinLeftConditional(const std::string& name, const NodeConfig& config, Robot* robot): SyncActionNode(name, config), _robot(robot) {
    _leftMotor = _robot->getMotor("left wheel motor");
    _rightMotor = _robot->getMotor("right wheel motor");
    _camera = _robot->getCamera("pioneer_camera");
    _camera->recognitionEnable(50);
}

PortsList SpinLeftConditional::providedPorts() { return {}; }

NodeStatus SpinLeftConditional::tick() {
    _leftMotor->setPosition(INFINITY);
    _rightMotor->setPosition(INFINITY);
    _leftMotor->setVelocity(-MAX_SPEED);
    _rightMotor->setVelocity(MAX_SPEED);
    return NodeStatus::FAILURE;
}

/////////////////

CheckIfVisible::CheckIfVisible(const std::string& name, const NodeConfig& config, Robot* robot): SyncActionNode(name, config), _robot(robot) {
    _camera = _robot->getCamera("pioneer_camera");
    _camera->recognitionEnable(50);
}

PortsList CheckIfVisible::providedPorts() { return {}; }

NodeStatus CheckIfVisible::tick() {
    if (_camera->getRecognitionNumberOfObjects() > 0) {
        return NodeStatus::SUCCESS;
    }

    return NodeStatus::FAILURE;
}

////////////////

