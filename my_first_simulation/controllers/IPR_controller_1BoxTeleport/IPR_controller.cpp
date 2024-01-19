// webots libs
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Camera.hpp>

// BT libs
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

// other libs
#include <fstream>
#include <string>
#include <stdlib.h>
#include <time.h>

#include "common.h"
#include "IPR_nodes.h"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace BT;

#define MAX_SPEED 1.1908

// extern variables
bool hasGrabbedOnce = false;
bool hasDroppedOnce = false;
bool hasFoundOnce = false;
bool timeBonus = false;
bool finishedObjectives = false;
bool finishedObjectivesAgain = false;


int main(int argc, char **argv) {
  // seed rand
  srand (time(NULL));

  // create the Robot instance.
  Supervisor *robot = new Supervisor();
  
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // init motors
  Motor *base = robot->getMotor("base");
  Motor *forearm = robot->getMotor("forearm");
  Motor *left_gripper = robot->getMotor("gripper::left");
  Motor *right_gripper = robot->getMotor("gripper::right");
  Motor *rotational_wrist = robot->getMotor("rotational_wrist");
  Motor *upperarm = robot->getMotor("upperarm");
  Motor *wrist = robot->getMotor("wrist");
  
  base->setVelocity(MAX_SPEED);
  forearm->setVelocity(MAX_SPEED);
  left_gripper->setVelocity(MAX_SPEED);
  right_gripper->setVelocity(MAX_SPEED);
  rotational_wrist->setVelocity(MAX_SPEED);
  upperarm->setVelocity(MAX_SPEED);
  wrist->setVelocity(MAX_SPEED);
  
  // init sensors
  PositionSensor *base_sensor = robot->getPositionSensor("base_sensor");
  PositionSensor *forearm_sensor = robot->getPositionSensor("forearm_sensor");
  PositionSensor *left_gripper_sensor = robot->getPositionSensor("gripper::left_sensor");
  PositionSensor *right_gripper_sensor = robot->getPositionSensor("gripper::right_sensor");
  PositionSensor *rotational_wrist_sensor = robot->getPositionSensor("rotational_wrist_sensor");
  PositionSensor *upperarm_sensor = robot->getPositionSensor("upperarm_sensor");
  PositionSensor *wrist_sensor = robot->getPositionSensor("wrist_sensor");
  
  base_sensor->enable(timeStep);
  forearm_sensor->enable(timeStep);
  left_gripper_sensor->enable(timeStep);
  right_gripper_sensor->enable(timeStep);
  rotational_wrist_sensor->enable(timeStep);
  upperarm_sensor->enable(timeStep);
  wrist_sensor->enable(timeStep);
  
  TouchSensor *ts1 = robot->getTouchSensor("ts1");
  TouchSensor *ts3 = robot->getTouchSensor("ts3");
  
  ts1->enable(timeStep);
  ts3->enable(timeStep);
  
  Camera *camera = robot->getCamera("camera");
  // camera->enable(timeStep);
  camera->recognitionEnable(timeStep);

  // track iteration and generation
  std::ifstream iterationFile("../supervisor/iteration.txt");
  std::ifstream generationFile("../supervisor/generation.txt");
  int iterationCount;
  int generationCount;
  iterationFile >> iterationCount;
  generationFile >> generationCount;
  
  
  
  // BT init
  BehaviorTreeFactory factory;
  
  factory.registerNodeType<grabBox>("grabBox", robot);
  factory.registerNodeType<dropBox>("dropBox", robot);
  factory.registerNodeType<findBox>("findBox", robot);
  factory.registerNodeType<checkIfFinished>("checkIfFinished", robot);

  
  std::string treePath = "../supervisor/BTs/BT_" + std::to_string(iterationCount - 1) + ".xml";
  auto tree = factory.createTreeFromFile("./IPR_testTree.xml");
  // auto tree = factory.createTreeFromFile(treePath);
  
  
  // init fitness
  int fitness = 50;

  
  int grabScore = 100;
  int dropScore = 300;
  int findScore = 50;
  
  // init objectives
  Node *solid = robot->getFromDef("OBJECTIVE_1");
  
  bool robotThinksFinished = false;

  
  // testing
  
  
  while (robot->step(timeStep) != -1) {
    // testing
    

    
    // Get the position of the solid
    const double *position = solid->getPosition();
    
    // std::cout << position[0] << " " << position[1] << " " << position[2] << std::endl;
    
    NodeStatus treeStatus = NodeStatus::FAILURE;
    
    if (!robotThinksFinished) {
      treeStatus = tree.rootNode()->executeTick();
    }
    
    
    if (treeStatus == NodeStatus::SUCCESS) {
      robotThinksFinished = true;
    }
    
    // if placed in target position
    if ((position[0] < 0.05 && position[0] > 0) && (position[1] < 0.35 && position[1] > 0.25) && (position[2] < 0.05 && position[2] > 0)) {
      
      // once robot finishes, 50% chance for box to respawn at initial location
      if (!finishedObjectives) {
        int randNum = generationCount % 2;
        
        if (randNum == 1) { // restart box
          double targetPosition[3] = {0.03, -0.3, 0.0349498};
          solid->getField("translation")->setSFVec3f(targetPosition);
          
          double targetRotation[4] = {0, 1, 0, 0};
          solid->getField("rotation")->setSFRotation(targetRotation);
          
          double targetVelocity[6] = {0,0,0,0,0,0};
          solid->setVelocity(targetVelocity);
        } else { // dont restart box
          double targetVelocity[6] = {0,0,0,0,0,0};
          solid->setVelocity(targetVelocity);
    
          double targetPosition[3] = {-1.79, -0.0299998, 0.5};
          solid->getField("translation")->setSFVec3f(targetPosition);
          
          finishedObjectivesAgain = true;
        }

      } else { // if robot completes second time
        double targetVelocity[6] = {0,0,0,0,0,0};
        solid->setVelocity(targetVelocity);
  
        double targetPosition[3] = {-1.79, -0.0299998, 0.5};
        solid->getField("translation")->setSFVec3f(targetPosition);
        
        finishedObjectivesAgain = true;
      }
      
      finishedObjectives = true;
      
    } 
    // if dropped outside of target position
    else if (position[0] > -1 && position[2] < 0.04 && !(position[0] < 0.06 && position[0] > 0 && position[1] < -0.26 && position[1] > -0.32)) {
      
      
      double targetPosition[3] = {0.03, -0.3, 0.0349498};
      solid->getField("translation")->setSFVec3f(targetPosition);
      
      double targetRotation[4] = {0, 1, 0, 0};
      solid->getField("rotation")->setSFRotation(targetRotation);

      
      double targetVelocity[6] = {0,0,0,0,0,0};
      solid->setVelocity(targetVelocity);
    }
    
    // check if time bonus applicable
    // to get time bonus: robot needs to think task is finished, the task needs to actually be finished, and the time must be within bounds
    if (robotThinksFinished && robot->getTime() < 34 && finishedObjectivesAgain) {
      timeBonus = true;
    }
    
    
    // if time limit exceeded
    if (robot->getTime() > 40) {
      break;
    }
    
    
  };

  // Enter here exit cleanup code.
  
  if (hasFoundOnce) {
    // std::cout << "found\n";
    fitness += findScore;
  }
  
  if (hasGrabbedOnce) {
    // std::cout << "grabbed\n";
    fitness += grabScore;
  }
  
  if (finishedObjectives) {
    // std::cout << "finished once\n";
    fitness += dropScore;
  }
  
  if (finishedObjectivesAgain) {
    // std::cout << "finished twice\n";
    fitness += dropScore;
  }

  if (timeBonus) {
    fitness += 100;
  }
  
  std::cout << fitness << std::endl;
  
  
  std::ofstream fitnessFile("../fitness.txt");
  fitnessFile << fitness;
  fitnessFile.close();

  delete robot;
  return 0;
}
