from controller import Robot, Motor

# Time step in milliseconds
TIME_STEP = 64

# Create the Robot instance
robot = Robot()

# Get the motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Set motor positions to infinity for velocity control
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Set motor velocity to 10% of MAX_SPEED
MAX_SPEED = 6.28
left_motor.setVelocity(1 * MAX_SPEED)
right_motor.setVelocity(1 * MAX_SPEED)

# Main loop
while robot.step(TIME_STEP) != -1:
    pass

# Clean up and exit
robot.cleanup()
