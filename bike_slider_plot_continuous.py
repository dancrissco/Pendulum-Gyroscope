
import pybullet as p
import time
import pybullet_data
import math
import matplotlib.pyplot as plt

# Connect and setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Green ground
green_plane = p.createCollisionShape(p.GEOM_BOX, halfExtents=[5, 5, 0.01])
green_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[5, 5, 0.01], rgbaColor=[0.2, 0.8, 0.2, 1])
p.createMultiBody(baseCollisionShapeIndex=green_plane,
                  baseVisualShapeIndex=green_visual,
                  basePosition=[0, 0, -0.01])

# Load the colored bike
bike = p.loadURDF("bicycle_colored_verified.urdf", [0, 0, 0.05], p.getQuaternionFromEuler([0, 0, 0]),
                  flags=p.URDF_USE_INERTIA_FROM_FILE)

# Let it settle
for _ in range(100):
    p.stepSimulation()
    time.sleep(1./240.)

# Joint indices
STEERING = 0
FRONT_WHEEL = 3
PENDULUM = 5
REAR_WHEEL = 9

# GUI sliders
steering_slider = p.addUserDebugParameter("Steering Angle (rad)", -1.0, 1.0, 0.4)
velocity_slider = p.addUserDebugParameter("Drive Velocity", -50.0, 50.0, 10.0)

# Control parameters
Kp = -400.0
max_reaction_speed = 300
reaction_force = 50

# Data logging
log_time = []
log_velocity = []

# Run simulation (limited logging, infinite sim)
start_time = time.time()
log_limit = 2000
step = 0

while True:
    t = time.time() - start_time

    # Get orientation
    _, orn = p.getBasePositionAndOrientation(bike)
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    tilt = pitch

    # Compute reaction wheel velocity
    reaction_velocity = Kp * tilt
    reaction_velocity = max(-max_reaction_speed, min(max_reaction_speed, reaction_velocity))

    # Apply control
    p.setJointMotorControl2(bike, PENDULUM, p.VELOCITY_CONTROL, targetVelocity=reaction_velocity, force=reaction_force)

    # Read sliders
    steer_val = p.readUserDebugParameter(steering_slider)
    drive_vel = p.readUserDebugParameter(velocity_slider)

    # Apply steering and drive
    p.setJointMotorControl2(bike, STEERING, p.POSITION_CONTROL, targetPosition=steer_val, force=5)
    p.setJointMotorControl2(bike, FRONT_WHEEL, p.VELOCITY_CONTROL, targetVelocity=drive_vel, force=10)
    p.setJointMotorControl2(bike, REAR_WHEEL, p.VELOCITY_CONTROL, targetVelocity=drive_vel, force=10)

    # Log for first N steps
    if step < log_limit:
        log_time.append(t)
        log_velocity.append(reaction_velocity)

    # After logging period ends, save plot once
    if step == log_limit:
        plt.figure(figsize=(10, 4))
        plt.plot(log_time, log_velocity, label="Reaction Wheel Velocity", color="orange")
        plt.title("Reaction Wheel Velocity Over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (rad/s)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig("reaction_velocity_plot.png")
        print("📊 Plot saved as reaction_velocity_plot.png")

    step += 1
    p.stepSimulation()
    time.sleep(1./240.)
