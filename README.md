# ME405-Project-Documentation
This repository stores the code for our romi project that was developed over the 10 weeks of the Winter 2026 quarter at California Polytechnic State University, San Luis Obispo. 

Prior to the final project, we had 7 labs to build and program Romi to get it into a state to complete the final project. These labs ranged in complexity and scope but all worked to advance our knowledge of our hardware as well as micropython. All the code and the reports generated for these labs can be viewed in this repository as well. The labs we completed will be briefly described here:

Lab 0x00 - Interrupt Callbacks and ADC Reading
  We utilized an analog to digital converter, commonly referred to as an ADC, to measure the capacitor voltage in an RC circuit. We generated a step-response plot showing the ADC voltage plotted against time and determined a time constant based on the step response data. This value was compared to one calculated from the component values in our RC circuit.
  
Lab 0x01 - Romi Hardware Confirmation
  We assembled our romi robots and wrote some simple code to confirm that the motors and encoders were working.
  
Lab 0x02 - Writing Hardware Drivers
  We developed a pair of driver files to help interface with PMDC motors and quadrature encoders.
  
Lab 0x03 - Closed Loop Control and the Scheduler
  We incorporated the hardware drivers we wrote the previous week with a set of tasks running cooperatively using a priority scheduler. Our primary goal was to use the hardware drivers from the last lab along with new scheduler libraries provided by Dr. Ridgley to create a closed-loop velocity controller to operate the gearmotors on the Romi chassis; a secondary goal of our assignment was to practice working with tasks, shares, and the scheduler.
  
Lab 0x04 - UI Design and Automated Data Collection
  In this lab we augmented our program from Lab 0x03 to facilitate automatic data collection of step responses. We modified our code running on the Nucleo so that step responses could be triggered through interaction with the serial port. We adjusted our user interface to allow a setpoint and set of gains to be entered before each step response runs. We tested this manually using PuTTY and then switched over to using a Python script running on our computer that communicated over serial. This script emulated the commands that would normally typed into PuTTY, so that the data could be captured by our script and plotted automatically.
  
Lab 0x05 - Line Following
  We wrote a new hardware driver for our line sensors and got Romi to drive in a 60 cm diameter circle with closed-loop actuation using feedback from the line sensors.
  
Lab 0x06 - State Estimation
  TWe wrote a new hardware driver for our BNO055 inertial measurement unit (IMU) and then used the data to develop a state-estimation algorithm that allows us to combine data from our sensors with our model of Romi from HW 0x03 to better predict the true state of Romi.

Misc Files contains extra memos and files that were used througout the project but were not necessary for any one lab deliverable.

Task_Diagram and Wiring_Diagram contain files generated for this readme in multiple formats.

Term Project contains the final code necessary to run the course with our Romi.

The ROMI Pin Configuration excel spreadsheet provides a pinout of our wiring with romi and associated boards.
