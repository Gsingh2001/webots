from controller import Robot, DistanceSensor, Motor

# Time step of the simulation in [ms]
TIME_STEP = 64

MAX_SPEED = 6.28  # Max speed of the motors

# Create the Robot instance
robot = Robot()

# Initialize sensor variables
ps = []
ps_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]

# Initialize distance sensors
for i in range(8):
    ps.append(robot.getDevice(ps_names[i]))
    ps[i].enable(TIME_STEP)

# Initialize motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Set motor positions to infinity to allow for continuous rotation
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Initialize motor velocities to 0
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Main feedback loop
while robot.step(TIME_STEP) != -1:
    # Read sensor values
    ps_values = [ps[i].getValue() for i in range(8)]

    # Detect obstacles
    right_obstacle = ps_values[0] > 80.0 or ps_values[1] > 80.0 or ps_values[2] > 80.0
    left_obstacle = ps_values[5] > 80.0 or ps_values[6] > 80.0 or ps_values[7] > 80.0

    # Initialize motor speeds at 50% of max speed
    left_speed = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED

    # Modify speeds based on obstacle detection
    if left_obstacle:
        # Turn right
        left_speed = 0.5 * MAX_SPEED
        right_speed = -0.5 * MAX_SPEED
    elif right_obstacle:
        # Turn left
        left_speed = -0.5 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED

    # Set motor velocities
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
