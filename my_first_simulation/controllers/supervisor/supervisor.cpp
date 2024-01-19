#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/TouchSensor.hpp>

#include <fstream>
#include <iostream>

// BT files
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

// other libs
#include <fstream>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <stack>

#include "testNodes.h"


#define TIME_STEP 32

using namespace webots;
using namespace BT;

/*
nodeTypes are: 
  FORWARD = 0,
  SPINLEFT = 1,
  SPINRIGHT = 2,
  BACKWARD = 3,
  CHECKDS0 = 4,
  CHECKDS15 = 5,
  CHECKDS13 = 6,
  CHECKDS2 = 7,
  SEQUENCE = 8,
  FALLBACK = 9
*/

int numNodes = 9;

struct myBTNode {
  int nodeType;
  std::vector<myBTNode*> children;

  myBTNode(int nodeType) {
    this->nodeType = nodeType;
  }

  ~myBTNode() {
    for (size_t i = 0;i < this->children.size();i++) {
      delete this->children[i];
    }
  }

  std::string generateXML() {
    std::string result;

    std::string childXML;
    switch (this->nodeType) {
    case 0:
      result = "<Sequence>";

      for (size_t i = 0;i < this->children.size();i++) {
        childXML += this->children[i]->generateXML();
      }

      result += childXML;
      result += "</Sequence>";
    break;
    case 1:
      result = "<Fallback>";

      for (size_t i = 0;i < this->children.size();i++) {
        childXML += this->children[i]->generateXML();
      }

      result += childXML;
      result += "</Fallback>";
    break;
    case 2:
      result = "<findBox/>";
    break;
    case 3:
      result = "<grabBox/>";
    break;
    case 4:
      result = "<dropBox/>";
    break;
    case 5:
      result = "<checkIfFinished/>";
    break;
    case 6:
      result = "<findBoxBlue/>";
    break;
    case 7:
      result = "<grabBoxBlue/>";
    break;
    case 8:
      result = "<checkIfFinishedBlue/>";
    break;
    }

    return result;
  }

};

myBTNode* createRandomTree(bool initial) {

  int controlNodePercentChance = 15;

  int randNum = rand() % 100;
  int num = 0;
  
  if (randNum < controlNodePercentChance) {
  // std::cout << "making control!\n";
    if (rand() % 2 == 1) { // make sequence node
      num = 0;
    } else { // make fallback node
      num = 1;
    }
  } else {
    num = rand() % (numNodes-2) + 2;
  }
  
  // if root node, make sequence
  if (initial) {
    initial = false;
    num = 0;
  }
  
  myBTNode* root = new myBTNode(num);
  
  // if action node
  if (num > 1) {
    return root;
  }
  
  // otherwise if control node
  int maxChildren = 10;
  int numChildren = rand() % maxChildren + 1;
  for (int i = 0;i < numChildren;i++) {
    root->children.push_back(createRandomTree(initial));
  }

  return root;
}

void mutateTree(myBTNode* root) {
  
  int mutationRate = 20; // one in (mutationRate) chance of mutating
  int randNum = rand() % mutationRate + 1;

  bool changedToControl = false;

  if (randNum == mutationRate) {
    
    if (root->nodeType == 0) { // if sequence, make fallback and add/remove children
      
      int addSubtree = rand() % 2;
      if (addSubtree > 0) { // add child
        root->children.push_back(createRandomTree(false));
      } else { // remove random child
        if (root->children.size() > 1) {
        
          // std::cout << "removed!\n";
          int selectedChild = rand() % root->children.size();
          delete root->children[selectedChild];
          root->children.erase(root->children.begin() + selectedChild);
        }
      }
      
    } else if (root->nodeType == 1) { // if fallback, vice versa
      
      int addSubtree = rand() % 2;
      if (addSubtree > 0) { // add child
        root->children.push_back(createRandomTree(false));
      } else { // remove random child
        if (root->children.size() > 1) {
        
          // std::cout << "removed!\n";
          int selectedChild = rand() % root->children.size();
          delete root->children[selectedChild];
          root->children.erase(root->children.begin() + selectedChild);
        }
      }
      
    } else {
    
      int newType = rand() % numNodes;
      root->nodeType = newType;
      
      // if control node
      if (root->nodeType < 2) {
        changedToControl = true;
        root->children.push_back(createRandomTree(false));
      }

    }
    
  }

  if (!changedToControl) {
    for (size_t i = 0;i < root->children.size();i++) {
      mutateTree(root->children[i]);
    }
  }
  
}

void crossover(myBTNode* root1, myBTNode* root2) {
    // edge case
    if (root1->children.size() == 0 || root2->children.size() == 0) {
        return;
    }

    // select random subtree from each tree
    std::vector<myBTNode*> list1;
    std::vector<myBTNode*> list2;

    // iterative DFS
    std::stack<myBTNode*> myStack;

    myStack.push(root1);
    while (!myStack.empty()) {
        myBTNode* current = myStack.top();
        myStack.pop();
        for (size_t i = 0;i < current->children.size();i++) {
            myStack.push(current->children[i]);
        }

        if (current->children.size() > 0) {
            list1.push_back(current);
        }
        
    }

    myStack.push(root2);
    while (!myStack.empty()) {
        myBTNode* current = myStack.top();
        myStack.pop();
        for (size_t i = 0;i < current->children.size();i++) {
            myStack.push(current->children[i]);
        }
        
        if (current->children.size() > 0) {
            list2.push_back(current);
        }
    }
    
    
    int randNum; 
    if (list1.size() - 1 == 0) {
        randNum = 0;
    } else {
        randNum = rand() % (list1.size() - 1);
    }

    myBTNode* parent1 = list1[randNum];
    myBTNode* subTree1;
    int index1 = rand() % parent1->children.size();
    subTree1 = parent1->children[index1];

    if (list2.size() - 1 == 0) {
        randNum = 0;
    } else {
        randNum = rand() % (list2.size() - 1);
    }

    myBTNode* parent2 = list2[randNum];
    myBTNode* subTree2;
    int index2 = rand() % parent2->children.size();
    subTree2 = parent2->children[index2];
    
    // swap subtrees
    parent1->children[index1] = subTree2;
    parent2->children[index2] = subTree1;
}


myBTNode* rouletteParent(const std::vector<myBTNode*>& myTrees, int* fitnessScores, int totalFitness) {
  int randomNum = rand() % totalFitness;
  int cumulativeFitness = 0;

  for (size_t i = 0; i < myTrees.size(); i++) {
    cumulativeFitness += fitnessScores[i];
    if (randomNum < cumulativeFitness) {
      return myTrees[i];
    }
  }

  std::cout << "roulette failed\n";
  return nullptr;
}



myBTNode* copyTree(myBTNode* root) {
  myBTNode* newNode = new myBTNode(root->nodeType);
  for (size_t i = 0;i < root->children.size();i++) {
    newNode->children.push_back(copyTree(root->children[i]));
  }
  return newNode;
}

int getTreeSize(myBTNode* root) {
  if (root->children.size() == 0) {
    return 1;
  }

  int treeSize = 1;
  for (size_t i = 0;i < root->children.size();i++) {
    treeSize += getTreeSize(root->children[i]);
  }
  return treeSize;
}

int main(int argc, char **argv) {
  // properties
  int taskNumber = 3;
  
  if (taskNumber == 1) {
    numNodes = 5;
  } else if (taskNumber == 2) {
    numNodes = 6;
  } else if (taskNumber == 3) {
    numNodes = 9;
  }
  
  int generationSize = 10; // needs to be even number
  
  int timeRestriction = 25; // 25 seconds per cube, teleporting cubes count twice, needs to be changed in other controllers too
  std::ofstream timeFile("time.txt");
  timeFile << timeRestriction;
  timeFile.close();

  int maxTreeSize = 30;

  // seed rand
  srand (time(NULL));
  
  Supervisor *supervisor = new Supervisor(); // create Supervisor instance
  
  Node* myRobot = supervisor->getFromDef("IPR_robot");
  
  
  int iterationCount = 1;
  int generationCount = 1;
  
  std::ofstream iterationFile("iteration.txt");
  std::ofstream generationFile("generation.txt");
  iterationFile << iterationCount;
  generationFile << generationCount;
  iterationFile.close();
  generationFile.close();
  
  // data neatness
  std::ofstream fittestDataFile("fittest_data.txt", std::ios::app);
  fittestDataFile << std::endl;
  fittestDataFile.close();
  
  std::ofstream dataFile("data.txt", std::ios::app);
  dataFile << std::endl;
  dataFile.close();
  
  // create first generation
  std::vector<myBTNode*> myTrees;
  for (int i = 0;i < generationSize;i++) {
    myBTNode* currTree = createRandomTree(true);
    myTrees.push_back(currTree);

    // write into XML file
    std::string treeName = "BTs/BT_" + std::to_string(i) + ".xml";
    std::ofstream treeFile(treeName);
    std::string treeXML = "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\"><Sequence name=\"root_sequence\">   " + currTree->generateXML() + "   </Sequence></BehaviorTree></root>";
    treeFile << treeXML;
    treeFile.close();
  }
  
  // fitness scores
  int* fitnessScores = new int[generationSize];
  
  for (int i = 0;i < generationSize;i++) {
    fitnessScores[i] = 0;
  }
  
  // determine whether box will teleport or not for first generation
  int blueTeleports = rand() % 2;
  int redTeleports = rand() % 2;
  
  if (taskNumber == 1) {
    blueTeleports = -1;
    redTeleports = 0;
  } else if (taskNumber == 2) {
    blueTeleports = -1;
    redTeleports = generationCount % 2;
  }
  
  std::ofstream blueFile("blueTeleports.txt");
  std::ofstream redFile("redTeleports.txt");
  blueFile << blueTeleports;
  redFile << redTeleports;
  blueFile.close();
  redFile.close();

  // reset simulation
  supervisor->simulationReset();
  myRobot->restartController();

  while (supervisor->step(TIME_STEP) != -1) {
    // do other stuff
  
    
    
    // new iteration
    if (supervisor->getTime() > (2 + blueTeleports + redTeleports)*timeRestriction + 11) {
        // do stuff
        
        
        
        int fitness;
        std::ifstream fitnessFile("../fitness.txt");
        fitnessFile >> fitness;
        int treeSize = getTreeSize(myTrees[iterationCount-1]);
        fitness -= 3*treeSize;
        if (fitness < 0 || treeSize > maxTreeSize) {
          fitness = 0;
        }
        fitnessScores[iterationCount-1] = fitness;
        
        // console log
        std::cout << "FINISHED: Iteration " << iterationCount << ", Generation " << generationCount << ", Fitness: " << fitness << std::endl;


        // track iteration/generation
        iterationCount++;
        
        // new generation
        if (iterationCount > generationSize) {
            
            iterationCount = 1;
            generationCount++;
            
            std::ofstream generationFile("generation.txt");
            generationFile << generationCount;
            generationFile.close();
            
            // determine whether box will teleport or not for current generation
            blueTeleports = rand() % 2;
            redTeleports = rand() % 2;
            
            if (taskNumber == 1) {
              blueTeleports = -1;
              redTeleports = 0;
            } else if (taskNumber == 2) {
              blueTeleports = -1;
              redTeleports = generationCount % 2;
            }
            
            std::ofstream blueFile("blueTeleports.txt");
            std::ofstream redFile("redTeleports.txt");
            blueFile << blueTeleports;
            redFile << redTeleports;
            blueFile.close();
            redFile.close();
          
            // reset simulation
            supervisor->simulationReset();
            myRobot->restartController();
            
            // perform natural selection
            int totalFitness = 0;
            for (int i = 0;i < generationSize;i++) {
              totalFitness += fitnessScores[i];
            }
            
            // log average generation fitness
            std::ofstream dataFile("data.txt", std::ios::app);
            double averageFitness = totalFitness/generationSize;
            dataFile << std::to_string(averageFitness) << std::endl;
            dataFile.close();

            std::vector<myBTNode*> newTrees;
            for (int i = 0;i < generationSize/2;i++) {
              // select parents
              myBTNode* child1 = copyTree(rouletteParent(myTrees,fitnessScores,totalFitness));
              myBTNode* child2 = copyTree(rouletteParent(myTrees,fitnessScores,totalFitness));

              
              crossover(child1,child2);
              
              
              mutateTree(child1);
              mutateTree(child2);
              

              newTrees.push_back(child1);
              newTrees.push_back(child2);
              
            }
            
            // include previous fittest
            delete newTrees[0];
            int prevBestIndex = 0;
            int max = fitnessScores[0];
            for (int i = 0;i < generationSize;i++) {
              if (fitnessScores[i] > max) {
                prevBestIndex = i;
                max = fitnessScores[i];
              }
            }
            newTrees[0] = copyTree(myTrees[prevBestIndex]);
            
            // log max fitness
            std::ofstream fittestDataFile("fittest_data.txt", std::ios::app);
            fittestDataFile << max << std::endl;
            fittestDataFile.close();

            // kill parents
            for (size_t i = 0;i < myTrees.size();i++) {
              delete myTrees[i];

              // write into XML file
              std::string treeName = "BTs/BT_" + std::to_string(i) + ".xml";
              std::ofstream treeFile(treeName);
              std::string treeXML = "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\"><Sequence name=\"root_sequence\">   " + newTrees[i]->generateXML() + "   </Sequence></BehaviorTree></root>";
              treeFile << treeXML;
              treeFile.close();

            }

            myTrees = newTrees;

        }
        
        std::ofstream iterationFile("iteration.txt");
        iterationFile << iterationCount;
        iterationFile.close();

        
        
        // reset simulation
        supervisor->simulationReset();
        myRobot->restartController();
        
        
    }
  }
  
  // log fittest BT genes
  std::ofstream bestBTFile("fittest_BTs.txt", std::ios::app);
  bestBTFile << "Fitness: " << fitnessScores[0] << std::endl << myTrees[0]->generateXML() << std::endl << std::endl;
  bestBTFile.close();

  delete supervisor;

  return 0;
}
