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
bool hasFoundOnce = false;
bool timeBonus = false;

bool hasGrabbedOnceBlue = false;
bool hasFoundOnceBlue = false;


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
  
  std::ifstream blueFile("../supervisor/blueTeleports.txt");
  std::ifstream redFile("../supervisor/redTeleports.txt");
  int randNumRed;
  int randNumBlue;
  blueFile >> randNumRed;
  redFile >> randNumBlue;
  
  
  // BT init
  BehaviorTreeFactory factory;
  
  factory.registerNodeType<grabBox>("grabBox", robot);
  factory.registerNodeType<dropBox>("dropBox", robot);
  factory.registerNodeType<findBox>("findBox", robot);
  factory.registerNodeType<checkIfFinished>("checkIfFinished", robot);
  
  factory.registerNodeType<grabBoxBlue>("grabBoxBlue", robot);
  factory.registerNodeType<findBoxBlue>("findBoxBlue", robot);
  factory.registerNodeType<checkIfFinishedBlue>("checkIfFinishedBlue", robot);


  
  std::string treePath = "../supervisor/BTs/BT_" + std::to_string(iterationCount - 1) + ".xml";
  // auto tree = factory.createTreeFromFile("./IPR_testTree.xml");
  auto tree = factory.createTreeFromFile(treePath);
  
  
  // init fitness
  int fitness = 50;

  
  int grabScore = 100;
  int dropScore = 300;
  int findScore = 50;
  
  // init objectives
  Node *solid = robot->getFromDef("OBJECTIVE_1");
  Node *solidBlue = robot->getFromDef("OBJECTIVE_2");
  
  bool robotThinksFinished = false;
  int finishedCountRed = 0;
  int finishedCountBlue = 0;
  
  int timesDroppedRed = 0;
  int timesDroppedBlue = 0;
  
  int setTime;
  
  std::ifstream timeFile("../supervisor/time.txt");
  
  timeFile >> setTime;
  
  int timeRestriction = (2 + randNumRed + randNumBlue)*setTime;
  
  // testing
  
  
  while (robot->step(timeStep) != -1) {
    // testing
    
    
    
    // BT propagate tick
    NodeStatus treeStatus = NodeStatus::FAILURE;
    
    if (!robotThinksFinished) {
      treeStatus = tree.rootNode()->executeTick();
    }
    
    if (treeStatus == NodeStatus::SUCCESS) {
      // std::cout << "robot\n";
      // std::cout << robot->getTime() << std::endl;
      robotThinksFinished = true;
    }
    
    
    // Get the position of the solid - RED BOX
    const double *positionRed = solid->getPosition();
    
    // std::cout << position[0] << " " << position[1] << " " << position[2] << std::endl;
    
    // if placed in target position
    if ((positionRed[0] < 0.05 && positionRed[0] > 0) && (positionRed[1] < 0.35 && positionRed[1] > 0.25) && (positionRed[2] < 0.05 && positionRed[2] > 0)) {
      
      // once robot finishes for first time, 50% chance for box to respawn at initial location
      if (finishedCountRed == 0) {
        
        if (randNumRed == 1) { // restart box
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
          
          finishedCountRed++;
        }

      } else { // if robot completes second time
        double targetVelocity[6] = {0,0,0,0,0,0};
        solid->setVelocity(targetVelocity);
  
        double targetPosition[3] = {-1.79, -0.0299998, 0.5};
        solid->getField("translation")->setSFVec3f(targetPosition);
        
      }
      
      finishedCountRed++;
      timesDroppedRed++;
      
    } 
    // if dropped outside of target position
    else if (positionRed[0] > -1 && positionRed[2] < 0.04 && !(positionRed[0] < 0.06 && positionRed[0] > 0 && positionRed[1] < -0.26 && positionRed[1] > -0.32)) {
      
      
      double targetPosition[3] = {0.03, -0.3, 0.0349498};
      solid->getField("translation")->setSFVec3f(targetPosition);
      
      double targetRotation[4] = {0, 1, 0, 0};
      solid->getField("rotation")->setSFRotation(targetRotation);

      
      double targetVelocity[6] = {0,0,0,0,0,0};
      solid->setVelocity(targetVelocity);
    }

    // Get the position of the solidBlue - BLUE BOX
    const double *positionBlue = solidBlue->getPosition();
    
    // std::cout << position[0] << " " << position[1] << " " << position[2] << std::endl;
    
    // if placed in target position
    if ((positionBlue[0] < 0.05 && positionBlue[0] > 0) && (positionBlue[1] < 0.35 && positionBlue[1] > 0.25) && (positionBlue[2] < 0.05 && positionBlue[2] > 0)) {
      
      // once robot finishes for the first time, 50% chance for box to respawn at initial location
      if (finishedCountBlue == 0) {
        
        if (randNumBlue == 1) { // restart box
          double targetPosition[3] = {0.191447, -0.219493, 0.0349498};
          solidBlue->getField("translation")->setSFVec3f(targetPosition);
          
          double targetRotation[4] = {0.000, 0.000, 1.000, 0.785};
          solidBlue->getField("rotation")->setSFRotation(targetRotation);
          
          double targetVelocity[6] = {0,0,0,0,0,0};
          solidBlue->setVelocity(targetVelocity);
        } else { // dont restart box
          double targetVelocity[6] = {0,0,0,0,0,0};
          solidBlue->setVelocity(targetVelocity);
    
          double targetPosition[3] = {-1.79, -0.0299998, 0.5};
          solidBlue->getField("translation")->setSFVec3f(targetPosition);
          
          finishedCountBlue++;
        }

      } else { // if robot completes second time
        double targetVelocity[6] = {0,0,0,0,0,0};
        solidBlue->setVelocity(targetVelocity);
  
        double targetPosition[3] = {-1.79, -0.0299998, 0.5};
        solidBlue->getField("translation")->setSFVec3f(targetPosition);
        
      }
      
      finishedCountBlue++;
      timesDroppedBlue++;
      
    } 
    // if dropped outside of target position
    else if (positionBlue[0] > -1 && positionBlue[2] < 0.04 && !(positionBlue[0] < 0.212 && positionBlue[0] > 0.161 && positionBlue[1] < -0.189 && positionBlue[1] > -0.249)) {
            
      double targetPosition[3] = {0.191447, -0.219493, 0.0349498};
      solidBlue->getField("translation")->setSFVec3f(targetPosition);
      
      double targetRotation[4] = {0.000, 0.000, 1.000, 0.785};
      solidBlue->getField("rotation")->setSFRotation(targetRotation);

      
      double targetVelocity[6] = {0,0,0,0,0,0};
      solidBlue->setVelocity(targetVelocity);
    }
    
    
    
    
    
    // check if time bonus applicable
    // to get time bonus: robot needs to think task is finished, the task needs to actually be finished, and the time must be within bounds
    if (robotThinksFinished && robot->getTime() < timeRestriction && (2 + randNumRed + randNumBlue == timesDroppedBlue + timesDroppedRed)) {
      timeBonus = true;
    }
    
    
    // if time limit exceeded
    if (robot->getTime() > timeRestriction + 10) {
      break;
    }
    
    
    
    
  };

  // Enter here exit cleanup code.
  
  // std::cout << timesDroppedRed + timesDroppedBlue << " " << 2 + randNumRed + randNumBlue << std::endl;
  
  // std::cout << randNumRed << " " << randNumBlue << std::endl;
  
  if (hasFoundOnce) {
    // std::cout << "found red\n";
    fitness += findScore;
  }
  
  if (hasGrabbedOnce) {
    // std::cout << "grabbed red\n";
    fitness += grabScore;
  }
  
  if (hasFoundOnceBlue) {
    // std::cout << "found blue\n";
    fitness += findScore;
  }
  
  if (hasGrabbedOnceBlue) {
    // std::cout << "grabbed blue\n";
    fitness += grabScore;
  }
  
  for (int i = 0;i < finishedCountRed;i++) {
    fitness += dropScore;
  }
  
  for (int i = 0;i < finishedCountBlue;i++) {
    fitness += dropScore;
  }
  
  if (timeBonus) {
    // std::cout << "time\n";
    fitness += 100;
  }
  
  // std::cout << fitness << std::endl;
  
  
  std::ofstream fitnessFile("../fitness.txt");
  fitnessFile << fitness;
  fitnessFile.close();

  delete robot;
  return 0;
}
