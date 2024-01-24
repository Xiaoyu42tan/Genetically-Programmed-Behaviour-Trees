# Genetically programmed Behaviour Trees

This is a Webots Simulator project aimed to explore self-learning AI for robotics. 

**What is a behaviour tree?**

Behaviour trees are popular controllers for autonomous agents such as video game NPCs. They are well known in the game development industry for their reactivity (easily allowing autonomous agents to react appropriately to environmental changes) and modularity (complex trees can be abstracted to modules, allowing for code scalability and readability).  

**What does this project do?**

The growing complexity of robot behaviour has required a more robust means of controlling them autonomously. This project uses Webots simulator to simulate using behaviour trees to control a robot. Specifically, it implements genetic programming to automatically generate behaviour trees for a manipulator robot to perform a certain task.

There are 3 tasks of increasing difficulty that the robot must learn to do: 
1. Pick up a box and put it in a specified area
2. Same as task 1, but the box has a 50% chance of teleporting back to the original position. The robot must recognise this and attempt to put the box in the area until it stays in the area
3. Same as task 2, but now with two boxes. The robot must put them both in the specified area and make sure they stay there
These tasks were designed such that it tests the inherent reactivity of behaviour trees, as the robot must react to unpredictable environmental changes.

3 handcrafted behaviour trees were also made to complete these tasks. The genetically programmed behaviour trees performed on par with the handcrafted trees for the first two tasks, and **SURPASSED** the handcrafted tree on the third and most complex task. 
I measured performance based on:
1. What percentage of the task the robot completed (all genetically programmed trees finished in the end)
2. How fast the robot completed the task, and whether or not it KNOWS it completed the task
3. How many behaviour tree nodes it used to construct the tree

On the third task the genetic algorithm outperformed my handcrafted tree by using less tree nodes, and was on par with everything else. Here are the graphical results:

![image](https://github.com/Xiaoyu42tan/Genetically-Programmed-Behaviour-Trees/assets/114973467/44d0a368-e36e-4ace-bd92-273e39aabb47)

![image](https://github.com/Xiaoyu42tan/Genetically-Programmed-Behaviour-Trees/assets/114973467/884523ce-9466-4fae-afa8-5e723806239d)

![image](https://github.com/Xiaoyu42tan/Genetically-Programmed-Behaviour-Trees/assets/114973467/caa47033-5bdd-483b-9803-2ecefb5a1554)



**BUILDING:**

Nothing will work until you build the BT.CPP library from https://www.behaviortree.dev/

I built it from source

Once you have built it please link it through the Makefiles for the following controllers: IPR_controller, IPR_controller_1BoxNoTeleport, IPR_controller_1BoxTeleport.

The existing makefile was the setup for my computer with my personal directories, just change these paths to whatever directories you built in:

        INCLUDE = -I"C:\Users\tanyo\Documents\ExternalLibs\BTLIB\BehaviorTree.CPP\include"
        CXX_SOURCES = IPR_controller.cpp IPR_nodes.cpp

        LIBRARIES = -L"C:\Users\tanyo\Documents\ExternalLibs\BTLIB\build" -l libbehaviortree_cpp

**FUN STUFF:**

Once it works you can tweak GP parameters in the supervisor.cpp script.

You can tweak generationSize, taskNumber (1 to 3), and timeRestriction. Everything should work from there. The data is all collected in the supervisor script controller folder.
