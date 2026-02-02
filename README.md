# Desktop_robot_arm
Contains the joint control code for my desktop arm project

Code titled "joint_control.c". 

## How to use it:
- Put the code into Arduino IDE
- Connect Arduino UNO connected to the motors to the IDE
- Turn on the serial monitor
- Either type ALL a b c d, where "a,b,c" are joint angles, "d" is the step value that controls the gripper, i.e. the last motor.
  - This controls all the motors at once.
- Or to control the motors individually, type J1 a (J1 for joint 1, a for the angle you want it to travel to). Same goes for joint 2 & 3. For the gripper, type "G" and a step value you want

## Code description
This Arduino sketch implements a serial-controlled multi-axis robot arm controller using the AccelStepper library and a CNC Shield V3. It supports three rotational joints and one gripper, each driven by a stepper motor with acceleration-limited motion.
At a high level, the program:
- Maps CNC shield pins to each joint and the gripper
- Converts joint angles (degrees) into stepper motor steps using gear ratios
- Parses human-readable commands from the Serial Monitor
- Generates smooth, non-blocking motion for all motors simultaneously

### Motor & Hardware Abstraction
Each motor is represented by an AccelStepper object:

- Joint 1 → X-axis pins
- Joint 2 → A-axis pins (if supported by the CNC shield)
- Joint 3 → Z-axis pins
- Gripper → Y-axis pins

All motors are configured in DRIVER mode (STEP + DIR), with:
- Adjustable max speed and acceleration
- Microstepping support
- Independent motion profiles per joint

The CNC shield enable pin is controlled in software so all drivers can be enabled or disabled together.

### Angle-to-Step Conversion

Joint commands are given in degrees, but stepper motors move in steps.

The code converts angles to steps using:
- Motor step resolution (200 steps/rev)
- Microstepping factor
- Joint-specific gearbox ratios

This allows you to command motion in real joint angles, while the firmware handles the low-level math automatically.

### Serial Command Processing

The main control interface is the Arduino Serial Monitor.

The program:

- Reads incoming text line-by-line
- Tokenizes commands using spaces
- Validates numeric input (including negative values)
- Executes motion commands safely
  
All parsing is done with non-blocking logic, so motors continue moving smoothly while new commands are received.

### Coordinated & Independent Motion

The controller supports:
- Independent joint control (one motor at a time)
- Synchronized multi-joint motion (all joints move together)

When a command is received:

- Each motor is given a new target position using moveTo()
- Motion is executed gradually using run() inside the main loop
- Acceleration and speed limits prevent sudden or jerky motion
Because AccelStepper is non-blocking, all motors can move simultaneously without delays.

### Gripper Control & Direction Inversion

The gripper is controlled in step units rather than degrees.

Key features:

- Relative and absolute positioning
- Software-level direction inversion (independent of wiring)
- Gentler acceleration and speed for safer gripping
This makes the gripper flexible and robust even if the motor wiring or mechanical layout changes.

### Main Loop Behavior

The loop() function continuously:

- Listens for new serial input
- Processes complete commands when received
- Calls run() on every motor every cycle

This ensures:

- Smooth motion
- Responsive command handling
- No blocking delays
