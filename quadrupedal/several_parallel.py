import time
import numpy as np
import math
import pybullet as pb
import pybullet_data
import controls
import random
from robot import Quadrupedal
from multiprocessing import Process, Manager
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def generate_oval_trajectory(center, x_radius, z_radius, angle, jitter_frequency=0.05, jitter_magnitude=0.01):
    """
    Generate a point on an oval trajectory with random jitter applied occasionally to simulate a more realistic movement.

    :param center: Center point of the oval [x, z]
    :param x_radius: Radius along x-axis
    :param z_radius: Radius along z-axis
    :param angle: Current angle (in radians)
    :param jitter_frequency: Probability of applying jitter at each iteration (0.0 to 1.0)
    :param jitter_magnitude: Maximum range for jitter applied to x, y, z
    :return: [x, y, z] coordinates of the point with optional jitter
    """
    # Calculate the basic oval trajectory point
    x = center[0] + x_radius * math.cos(-angle)
    y = center[1]  # Assuming the center's y position doesn't change (static on the ground)
    z = center[2] + z_radius * math.sin(-angle)

    # Randomly decide if jitter should be applied
    if random.random() < jitter_frequency:
        jitter_x = random.uniform(-jitter_magnitude, jitter_magnitude)
        jitter_y = random.uniform(-0.5, 0.5) 
        jitter_z = random.uniform(-jitter_magnitude, jitter_magnitude)

        # Apply the jitter
        x += jitter_x
        y += jitter_y
        z += jitter_z

    if random.random() < jitter_frequency:
        jitter_y = 0.5

        # Apply the jitter
        y += jitter_y

    return x, y, z

def reset_robot(robot, i):
    """
    Resets the robot to its initial position, orientation, and velocities.

    :param robot: An instance of the Quadrupedal class.
    """
    # Reset the base position and orientation
    pb.resetBasePositionAndOrientation(robot._robotId, [0, i, 0.55], [0, 0, 0, 1])
    
    # Reset the base velocity
    pb.resetBaseVelocity(robot._robotId, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
    
    print("Robot has been reset!")

def run_simulation(robot_id, num_robots, use_gui=True, coord_log=None):
    """
    Run an independent simulation for each robot in a separate PyBullet client.
    Log the coordinates of each leg for plotting.
    """
    if use_gui:
        physics_client = pb.connect(pb.GUI)
    else:
        physics_client = pb.connect(pb.DIRECT) # Non-graphical version

    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.8)

    robots = []

    # Create the robot instances for the specific client
    for i in range(num_robots):
        robot = Quadrupedal(
            timeStep=1./240.,
            initialCoMheight=0.3,
            robotPATH="urdf/quadrupedal.urdf",
            startPosition=[0, i, 0.55],
            startOrientation=[0., 0., 0.],
            maxForce=12
        )
        robots.append(robot)

    # Oval trajectory parameters
    oval_center_rf = [0.2, -0.11, -0.2]  # Center of the oval path 
    oval_center_lh = [-0.2, 0.11, -0.2] 
    oval_center_lf = [0.2, 0.11, -0.2] 
    oval_center_rh = [-0.2, -0.11, -0.2]  

    initial_width = 0.1  # Radius along x-axis (horizontal stretch)
    initial_height = 0.01  # Radius along z-axis (vertical stretch)

    width_slider = pb.addUserDebugParameter("Width", 0.01, 0.5, initial_width)
    height_slider = pb.addUserDebugParameter("Height", 0.01, 0.1, initial_height)

    # Angle increment for faster or slower movement
    angle_step_slider = pb.addUserDebugParameter("Angle Step", 0.01, 1.0, 0.05)
    angle = 0.0

    try:
        while True:
            # Reset robot state if "R" key is pressed
            if controls.is_reset_key_pressed(): 
                for i in range(num_robots):
                    reset_robot(robots[i], i)

            if use_gui:
                # Get the angle step from the slider (how fast the robot moves)
                angle_step = pb.readUserDebugParameter(angle_step_slider)

                # Get the width and height from the sliders
                x_radius = pb.readUserDebugParameter(width_slider)
                z_radius = pb.readUserDebugParameter(height_slider)
            else:
                angle_step = 0.05
                x_radius = 0.1
                z_radius = 0.01

            angle += angle_step
            if angle > 2 * math.pi:
                angle -= 2 * math.pi

            for qdrp in robots:
                # Diagonal Pair 1: Front Right (RF) and Left Hind (LH)
                RF_x, RF_y, RF_z = generate_oval_trajectory(oval_center_rf, x_radius, z_radius, angle)
                LH_x, LH_y, LH_z = generate_oval_trajectory(oval_center_lh, x_radius, z_radius, angle)
                # Diagonal Pair 2: Front Left (LF) and Right Hind (RH) (phase-offset by π)
                LF_x, LF_y, LF_z = generate_oval_trajectory(oval_center_lf, x_radius, z_radius, angle + math.pi)
                RH_x, RH_y, RH_z = generate_oval_trajectory(oval_center_rh, x_radius, z_radius, angle + math.pi)

                targetPositionRF = np.array([0.2, -0.11, -0.2])  # Right Front
                targetPositionRH = np.array([-0.2, -0.11, -0.2])  # Right Hind
                targetPositionLF = np.array([0.2, 0.11, -0.2])    # Left Front
                targetPositionLH = np.array([-0.2, 0.11, -0.2])   # Left Hind
                
                # Update target positions
                targetPositionRF[0], targetPositionRF[1], targetPositionRF[2] = RF_x, RF_y, RF_z
                targetPositionLH[0], targetPositionLH[1], targetPositionLH[2] = LH_x, LH_y, LH_z
                targetPositionLF[0], targetPositionLF[1], targetPositionLF[2] = LF_x, LF_y, LF_z
                targetPositionRH[0], targetPositionRH[1], targetPositionRH[2] = RH_x, RH_y, RH_z

                # Inverse Kinematics
                RFjointPositions = qdrp.inverseKinematics(targetPositionRF, targetLeg=qdrp.legRF)
                LHjointPositions = qdrp.inverseKinematics(targetPositionLH, targetLeg=qdrp.legLH)
                LFjointPositions = qdrp.inverseKinematics(targetPositionLF, targetLeg=qdrp.legLF)
                RHjointPositions = qdrp.inverseKinematics(targetPositionRH, targetLeg=qdrp.legRH)

                # Set joint positions
                qdrp.legRF.setJointPositions(RFjointPositions)
                qdrp.legLH.setJointPositions(LHjointPositions)
                qdrp.legLF.setJointPositions(LFjointPositions)
                qdrp.legRH.setJointPositions(RHjointPositions)

                # Advance simulation
                qdrp.oneStep()
            
            # log the coordinates of RF leg



    except KeyboardInterrupt:
        print(f"Simulation {robot_id} stopped by user.")
    finally:
        pb.disconnect()


def plot_trajectories(coord_log, num_simulations):
    """
    Plot the trajectories of the robots in real-time.
    """
    fig, ax = plt.subplots()
    lines = []
    points = []

    # Initialize plot elements for each robot
    for robot_id in range(num_simulations):
        line, = ax.plot([], [], label=f"Robot {robot_id} RF", lw=2)  # Blue trajectory line
        point, = ax.plot([], [], 'ro')  # Red dot for the latest point
        lines.append(line)
        points.append(point)

    def init():
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_title("Real-Time Leg Trajectories")
        ax.set_xlabel("X")
        ax.set_ylabel("Z")
        ax.legend()
        return lines + points

    def update(frame):
        for robot_id in range(num_simulations):
            trajectory = coord_log[robot_id]["RF"]
            print(f"Robot {robot_id} RF: {trajectory}")
            if len(trajectory) > 0:
                x_data, y_data, z_data = zip(*trajectory)
                lines[robot_id].set_data(x_data, z_data)  # Update trajectory
                points[robot_id].set_data(x_data[-1], z_data[-1])  # Update red dot
            else:
                lines[robot_id].set_data([], [])
                points[robot_id].set_data([], [])
        return lines + points

    ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=50)
    plt.show()


def main():
    num_robots = 1 # Number of robots per simulation
    num_simulations = 2 # Number of parallel simulations

    manager = Manager()
    coord_log = manager.dict()


    processes = []

    # Run simulations in parallel for each robot
    for i in range(num_simulations):
        use_gui = (i == 0) # Only show the GUI for the first simulation
        p = Process(target=run_simulation, args=(i, num_robots, use_gui, coord_log))
        p.start()
        processes.append(p)
    
    # # Wait for all processes to finish
    # for p in processes:
    #     p.join()

    # Start plotting
    try:
        plot_trajectories(coord_log, num_simulations)
    finally:
        # # Ensure all processes terminate cleanly
        # for p in processes:
        #     p.terminate()
        p.join()

if __name__ == "__main__":
    main()
