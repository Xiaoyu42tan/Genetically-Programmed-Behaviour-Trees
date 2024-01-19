#include "IPR_nodes.h"
#include "common.h"

using namespace BT;
using namespace webots;

#define MAX_SPEED 1.1908

// grab box

grabBox::grabBox(const std::string& name, const NodeConfig& config, Supervisor* robot): StatefulActionNode(name, config), _robot(robot) {
    base = robot->getMotor("base");
    forearm = robot->getMotor("forearm");
    left_gripper = robot->getMotor("gripper::left");
    right_gripper = robot->getMotor("gripper::right");
    rotational_wrist = robot->getMotor("rotational_wrist");
    upperarm = robot->getMotor("upperarm");
    wrist = robot->getMotor("wrist");

    base_sensor = robot->getPositionSensor("base_sensor");
    forearm_sensor = robot->getPositionSensor("forearm_sensor");
    left_gripper_sensor = robot->getPositionSensor("gripper::left_sensor");
    right_gripper_sensor = robot->getPositionSensor("gripper::right_sensor");
    rotational_wrist_sensor = robot->getPositionSensor("rotational_wrist_sensor");
    upperarm_sensor = robot->getPositionSensor("upperarm_sensor");
    wrist_sensor = robot->getPositionSensor("wrist_sensor");

    ts1 = robot->getTouchSensor("ts1");
    ts3 = robot->getTouchSensor("ts3");

    camera = robot->getCamera("camera");
}

PortsList grabBox::providedPorts() { return {}; }

NodeStatus grabBox::onStart() {
    bool found = false;
    for (int i = 0;i < camera->getRecognitionNumberOfObjects();i++) {
        if (camera->getRecognitionObjects()[i].colors[0] == 1) {
            hasFoundOnce = true;
            found = true;
        }
    }

    if (!found) {
        return NodeStatus::FAILURE;
    }

    base->setPosition(0);
    forearm->setPosition(0);
    left_gripper->setPosition(-1);
    right_gripper->setPosition(1);
    rotational_wrist->setPosition(0);
    upperarm->setPosition(-1);
    wrist->setPosition(-2);

    counter = 0;
    
    failCounter = 0;

    tsCounter = 0;

    return NodeStatus::RUNNING;
    
    
}

NodeStatus grabBox::onRunning() {
    failCounter++;
    if (failCounter > 300) {
        return NodeStatus::FAILURE;
    }


    if (wrist_sensor->getValue() < -1.9 && wrist_sensor->getValue() > -2.1) {
        left_gripper->setPosition(0.1);
        right_gripper->setPosition(0.1);

        // once box is grabbed, reset position
        if (ts1->getValue() == 0 && ts3->getValue() == 0) {
            tsCounter++;
            if (tsCounter > 5) {
                base->setPosition(0);
                forearm->setPosition(0);
                rotational_wrist->setPosition(0);
                upperarm->setPosition(0);
                wrist->setPosition(0);
                counter++;
            }
        }

        
    }

    if (counter > 0) {
        counter++;
    }

    if (counter > 80) {
        if (camera->getRecognitionNumberOfObjects() == 0) {
            return NodeStatus::FAILURE;
        }
        hasGrabbedOnce = true;
        return NodeStatus::SUCCESS;
    }

    return NodeStatus::RUNNING;
}

void grabBox::onHalted() {
    
}

// drop box

dropBox::dropBox(const std::string& name, const NodeConfig& config, Supervisor* robot): StatefulActionNode(name, config), _robot(robot) {
    base = robot->getMotor("base");
    forearm = robot->getMotor("forearm");
    left_gripper = robot->getMotor("gripper::left");
    right_gripper = robot->getMotor("gripper::right");
    rotational_wrist = robot->getMotor("rotational_wrist");
    upperarm = robot->getMotor("upperarm");
    wrist = robot->getMotor("wrist");

    base_sensor = robot->getPositionSensor("base_sensor");
    forearm_sensor = robot->getPositionSensor("forearm_sensor");
    left_gripper_sensor = robot->getPositionSensor("gripper::left_sensor");
    right_gripper_sensor = robot->getPositionSensor("gripper::right_sensor");
    rotational_wrist_sensor = robot->getPositionSensor("rotational_wrist_sensor");
    upperarm_sensor = robot->getPositionSensor("upperarm_sensor");
    wrist_sensor = robot->getPositionSensor("wrist_sensor");

    ts1 = robot->getTouchSensor("ts1");
    ts3 = robot->getTouchSensor("ts3");
    
    camera = robot->getCamera("camera");
}

PortsList dropBox::providedPorts() { return {}; }

NodeStatus dropBox::onStart() {
    wasSuccessful = false;
    if (camera->getRecognitionNumberOfObjects() == 0) {
        return NodeStatus::FAILURE;
    }

    base->setPosition(3);
    forearm->setPosition(0);
    
    rotational_wrist->setPosition(0);
    
    
    counter = 0;

    return NodeStatus::RUNNING;
    
    
}

NodeStatus dropBox::onRunning() {

    if (base_sensor->getValue() > 1.5 && counter == 0) {
        upperarm->setPosition(-1);
        wrist->setPosition(-2);
    }

    if (counter == 0 && (base_sensor->getValue() < 3.1 && base_sensor->getValue() > 2.9) && (upperarm_sensor->getValue() > -1.1 && upperarm_sensor->getValue() < -0.9) && (wrist_sensor->getValue() > -2.1 && wrist_sensor->getValue() < -1.9)) {
        // releasing cube
        left_gripper->setPosition(-1);
        right_gripper->setPosition(1);

        if (left_gripper_sensor->getValue() > 0.8 && right_gripper_sensor->getValue() > 0.8) {
            // going to base position
            base->setPosition(0);
            forearm->setPosition(0);
            rotational_wrist->setPosition(0);
            upperarm->setPosition(0);
            wrist->setPosition(0);
            counter++;
        }
    }

    if (counter > 0) {
        counter++;
    }

    if (counter > 100) {
        if (finishedObjectives) {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    return NodeStatus::RUNNING;
}

void dropBox::onHalted() {
    
}

// find box

findBox::findBox(const std::string& name, const NodeConfig& config, Supervisor* robot): StatefulActionNode(name, config), _robot(robot) {
    base = robot->getMotor("base");
    forearm = robot->getMotor("forearm");
    left_gripper = robot->getMotor("gripper::left");
    right_gripper = robot->getMotor("gripper::right");
    rotational_wrist = robot->getMotor("rotational_wrist");
    upperarm = robot->getMotor("upperarm");
    wrist = robot->getMotor("wrist");

    base_sensor = robot->getPositionSensor("base_sensor");
    forearm_sensor = robot->getPositionSensor("forearm_sensor");
    left_gripper_sensor = robot->getPositionSensor("gripper::left_sensor");
    right_gripper_sensor = robot->getPositionSensor("gripper::right_sensor");
    rotational_wrist_sensor = robot->getPositionSensor("rotational_wrist_sensor");
    upperarm_sensor = robot->getPositionSensor("upperarm_sensor");
    wrist_sensor = robot->getPositionSensor("wrist_sensor");

    ts1 = robot->getTouchSensor("ts1");
    ts3 = robot->getTouchSensor("ts3");

    camera = robot->getCamera("camera");
}

PortsList findBox::providedPorts() { return {}; }

NodeStatus findBox::onStart() {
    counter = 0;
    for (int i = 0;i < camera->getRecognitionNumberOfObjects();i++) {
        if (camera->getRecognitionObjects()[i].colors[0] == 1) {
            hasFoundOnce = true;
            return NodeStatus::SUCCESS;
        }
    }


    forearm->setPosition(1);
    wrist->setPosition(-3.84);
    upperarm->setPosition(-0.5);
    base->setPosition(0);
    rotational_wrist->setPosition(0);

    return NodeStatus::RUNNING;
}

NodeStatus findBox::onRunning() {
    for (int i = 0;i < camera->getRecognitionNumberOfObjects();i++) {
        if (camera->getRecognitionObjects()[i].colors[0] == 1) {
            return NodeStatus::SUCCESS;
            hasFoundOnce = true;
        }
    }

    if (counter > 100) {
        return NodeStatus::FAILURE;
    }

    counter++;
    return NodeStatus::RUNNING;
}

void findBox::onHalted() {
    
}