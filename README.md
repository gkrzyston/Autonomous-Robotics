# Autonomous-Robotics

This repo contains sample code from my University of Michigan Autonomous Robotics course final project.

Goals of this project:
• Design and implement system to predict the terrain being driven on by a simple robot.
• Implement firmware and motor control techniques to control robot movement.
• Implement a localization and mapping system using LiDAR.
• Collect vibration data using an accelerometer and I2C communication, and communicate between different chips using JSON and TCP/IP.
• Use multithreading in C/C++ for parallel programming.

The included code is for our terrain prediction algorithm, which runs separately from the robot movement programs

Directories:
  i2c - Contains C/C++ files to receive / process vibration data from an accelerometer (get_frequency.c)
      - This Data is sent using TCP/IP to the driver program

  botlab - Driver program receives vibration and motor data and makes a prediction
         - Contains Python script to visualize results in a map
         - Occupany grid class allows map of terrains to be stored
