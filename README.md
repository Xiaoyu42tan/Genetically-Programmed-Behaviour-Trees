# Genetically programmed BTs

This is a Webots Simulator project

BUILDING:

Nothing will work until you build the BT.CPP library from https://www.behaviortree.dev/

I built it from source

Once you have built it please link it through the Makefiles for the following controllers: IPR_controller, IPR_controller_1BoxNoTeleport, IPR_controller_1BoxTeleport.

The existing makefile was the setup for my computer with my personal directories, just change these paths to whatever directories you built in:

        INCLUDE = -I"C:\Users\tanyo\Documents\ExternalLibs\BTLIB\BehaviorTree.CPP\include"
        CXX_SOURCES = IPR_controller.cpp IPR_nodes.cpp

        LIBRARIES = -L"C:\Users\tanyo\Documents\ExternalLibs\BTLIB\build" -l libbehaviortree_cpp

FUN STUFF:

Once it works you can tweak GP parameters in the supervisor.cpp script.

You can tweak generationSize, taskNumber (1 to 3), and timeRestriction. Everything should work from there. The data is all collected in the supervisor script controller folder.
