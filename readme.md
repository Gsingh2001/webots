# Webots Simulation: e-Puck Collision Avoidance

This is a Webots simulation project where an e-puck robot is programmed to avoid obstacles using its 8 infrared distance sensors. The controller allows the robot to move forward until it detects an obstacle, after which it turns to avoid the obstacle and continues moving.

## Table of Contents
- [Project Overview](#project-overview)
- [Getting Started](#getting-started)
- [Simulation Features](#simulation-features)
- [Controller Code](#controller-code)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Project Overview

This project demonstrates how to program an e-puck robot in Webots to perform basic obstacle avoidance. The robot:
- Uses 8 distance sensors located around its turret to detect obstacles.
- Moves forward until an obstacle is detected.
- Turns in the opposite direction of the detected obstacle and continues forward.

The controller is written in Python, utilizing Webots' API to control the robot’s sensors and actuators.

## Getting Started

### Prerequisites

Before running this project, make sure you have Webots installed. You can download Webots from the official website:

- [Webots Download](https://cyberbotics.com/)

### Clone this repository

Clone the repository to your local machine:

```bash
git clone https://github.com/yourusername/webots-epuck-collision-avoidance.git
```

### Open the Project in Webots

1. Launch Webots.
2. Open the project directory: `File > Open...` and select the `collision_avoidance.wbt` world file.

### Running the Simulation

1. Ensure the simulation is paused initially.
2. Press the "Run" button (real-time mode) to start the simulation.
3. Observe the e-puck robot moving forward and avoiding obstacles by turning away from detected obstacles.

## Simulation Features

- **Arena**: A simple rectangular arena with randomly placed obstacles (such as wooden boxes) that the robot must avoid.
- **Robot**: An e-puck robot equipped with 8 infrared distance sensors and differential wheels.
- **Controller**: A Python controller that makes the e-puck robot avoid obstacles by controlling its motor speeds based on sensor data.
- **Physics**: The simulation includes realistic physics for object interaction, including the movement and collisions of the robot.

## Controller Code

The controller for the e-puck robot is written in Python. The code is located in the `controllers/epuck_avoid_collision/epuck_avoid_collision.py` file. Here's an overview of the code:

### Controller Overview:

```python
from controller import Robot, DistanceSensor, Motor

# Time step in milliseconds
TIME_STEP = 64
MAX_SPEED = 6.28

# Create the Robot instance
robot = Robot()

# Initialize distance sensors
ps = []
ps_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(ps_names[i]))
    ps[i].enable(TIME_STEP)

# Initialize motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Main loop
while robot.step(TIME_STEP) != -1:
    # Read sensor values
    ps_values = [sensor.getValue() for sensor in ps]

    # Detect obstacles
    right_obstacle = ps_values[0] > 80.0 or ps_values[1] > 80.0 or ps_values[2] > 80.0
    left_obstacle = ps_values[5] > 80.0 or ps_values[6] > 80.0 or ps_values[7] > 80.0

    # Set motor speeds
    left_speed = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED

    if left_obstacle:
        # Turn right
        left_speed = 0.5 * MAX_SPEED
        right_speed = -0.5 * MAX_SPEED
    elif right_obstacle:
        # Turn left
        left_speed = -0.5 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED

    # Write motor velocities
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

# Clean up and exit
robot.cleanup()
```

### How the Code Works:

- **Sensors**: The robot reads values from its 8 distance sensors (`ps0` to `ps7`).
- **Obstacle Detection**: If the values from the front or side sensors are greater than a threshold (80.0), the robot detects an obstacle.
- **Motor Control**: The robot adjusts its wheel velocities based on the sensor data to either move forward or turn left/right to avoid obstacles.

## Troubleshooting

- **Missing DLL for Remote Control**: If you encounter the error related to `e-puck_bluetooth.dll`, try disabling Bluetooth remote control in the Webots scene tree or reinstall Webots to restore missing files.
  
- **Simulation Not Running**: Ensure that the controller is correctly associated with the e-puck robot, and the simulation is not paused.

## Contributing

Contributions are welcome! If you'd like to contribute to this project, feel free to fork the repository, make changes, and submit a pull request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---