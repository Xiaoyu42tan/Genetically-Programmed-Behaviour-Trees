// Webots library
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

// BT library
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

// other libraries
#include <fstream>
#include <unordered_set>
#include <string>

#include "pioneerNodes.h"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace BT;


#define MAX_SPEED 6.28


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  
  // init robot stuff
  // camera
  Camera* camera = robot->getCamera("pioneer_camera");
  camera->recognitionEnable(timeStep);
  
  // track iteration and generation
  /*
  std::ifstream iterationFile("../supervisor/iteration.txt");
  std::ifstream generationFile("../supervisor/generation.txt");
  int iterationCount;
  int generationCount;
  iterationFile >> iterationCount;
  generationFile >> generationCount;
  
  std::cout << "Iteration: " << iterationCount << std::endl;
  std::cout << "Generation: " << generationCount << std::endl;
  */

  // BT init  
  BehaviorTreeFactory factory;
  
  factory.registerNodeType<MoveForward>("MoveForward", robot);
  factory.registerNodeType<SpinLeft>("SpinLeft", robot);
  factory.registerNodeType<SpinRight>("SpinRight", robot);
  factory.registerNodeType<MoveBackward>("MoveBackward", robot);
  factory.registerNodeType<CheckDS0>("CheckDS0", robot);
  factory.registerNodeType<CheckDS15>("CheckDS15", robot);
  factory.registerNodeType<CheckDS13>("CheckDS13", robot);
  factory.registerNodeType<CheckDS2>("CheckDS2", robot);
  
  factory.registerNodeType<testForward>("testForward", robot);
  factory.registerNodeType<testSpin>("testSpin", robot);

  // factory.registerNodeType<CheckIfVisible>("CheckIfVisible", robot);

  
  // main tree
  // std::string treePath = "../supervisor/BT_" + std::to_string(iterationCount - 1) + ".xml";
  auto tree = factory.createTreeFromFile("./test_tree.xml");
  
  // tests
  // Camera* camera = robot->getCamera("pioneer_camera");
  // camera->recognitionEnable(50);
  
  // 
  // sensor->type = DistanceSensor::LASER;
  
  // init measurements
  // std::unordered_set<int> wallSegmentSet;
  
  // std::ofstream outputFile("exploration.txt", std::ios::app); // Open file in append mode
  
  // main simulation loop  
  while (robot->step(timeStep) != -1) {
    
    // "explore" walls
    /*
    const CameraRecognitionObject* objectsInView = camera->getRecognitionObjects();
    for (int i = 0;i < camera->getRecognitionNumberOfObjects();i++) {
        if (wallSegmentSet.find(objectsInView[i].id) == wallSegmentSet.end()) {
            wallSegmentSet.insert(objectsInView[i].id);
        }
    }
    */
    
    
    // propagate tick from root node
    NodeStatus treeStatus = tree.rootNode()->executeTick();
    
    // check status
    if (treeStatus == NodeStatus::SUCCESS) {
      std::cout << "robot finished job!\n";
      break;
    } 
    
    // else if (treeStatus == NodeStatus::FAILURE) {
      // std::cout << "robot failed job :(\n";
      // break;
    // }
    
    
  }

  // Enter here exit cleanup code.
  /*
  outputFile << "Iteration: " << iterationCount << " Generation: " << generationCount << " Walls Explored: " << wallSegmentSet.size() << std::endl;

  std::string fitFilename = "fitness_" + std::to_string(iterationCount - 1) + ".txt";
  std::ofstream fitnessFile(fitFilename);
  fitnessFile << wallSegmentSet.size();
  fitnessFile.close();
  */

  delete robot;
  return 0;
}
