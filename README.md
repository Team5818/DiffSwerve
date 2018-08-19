# Diff Swerve

# Some Background
This repository houses the code for Team 5818's Differential Swerve Drive prototype. A differential swerve drive is similar to a "classic" swerve drive, except it doesn't have dedicated motors for steering and throttle. Instead, two drive motors are linked through a mechanical differential, which allows for the module to either rotate or translate depending on the ratio of the individual motors' velocities. For more information on the mechanical setup, consult this thread: https://www.chiefdelphi.com/forums/showthread.php?threadid=166236

If you are looking for a guide on progamming a "standard" swerve drive, much of this code will still be applicable, but the code for controlling the individual modules will be different.

# Repository Structure
All of the actual robot code for this project is located in the directory `src/org/rivierarobotcs/`. There are two other interesting directories in this repo: `simulation/` and `src-tests/javacontrols/`. Both of these directories are remnants of our original intent to use state feedback and kalman filters to control the module. `simulation/`contains a MATLAB script for simulating a differential swerve module controller, while `src-tests/javacontrols/` does the same thing in Java using the Efficient Java Matrix Library (EJML) to take care of the linear algebra. None of the state feedback code has gone on the robot (yet), but writing the simulation code was an excellent learning experience. All state feedback code draws heavy inspiration from 971's 2017 robot code.

# Setup
This repository uses FRCGrantle, Team 5818's custom build system. FRCGrantle is a hybrid between Gradle and Ant. It uses Gradle tasks to fetch dependencies and automate setup, but the code still deploys using the Ant scripts provided by FIRST. To set up this repository:

* Clone the repository
* Navigate to the root directory of the repository and run `./gradlew eclipse`
* After the build finishes, import the project into Eclipse
* To deploy, click `Run As > WPILib Java Deploy` in Eclipse.
* Enjoy!
