import time
import numpy as np
import math
import pybullet as pb
import pybullet_data
import controls
import random
from robot import Quadrupedal
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from queue import Queue

def generate_oval_trajectory(center, x_radius, z_radius, angle, jitter_frequency=0.03, jitter_magnitude=0.01):
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

def run_sequential_simulation(num_simulations, use_gui, max_points=500):
    """
    Run simulations sequentially in the same PyBullet client.
    """
    if use_gui:
        physics_client = pb.connect(pb.GUI)
    else:
        physics_client = pb.connect(pb.DIRECT) # Non-graphical version

    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.8)

    # Store coordinates for all simulations
    all_coordinates = {}

    # Create the robot instances
    robots = []
    for i in range(num_simulations):
        robot = Quadrupedal(
            timeStep=1./240.,
            initialCoMheight=0.3,
            robotPATH="../urdf/quadrupedal.urdf",
            startPosition=[0, i, 0.55],
            startOrientation=[0., 0., 0.],
            maxForce=12
        )
        robots.append(robot)
        all_coordinates[i] = {
            'id': i,
            'x_coords': [],
            'y_coords': [],
            'z_coords': []
        }

    # Oval trajectory parameters
    oval_centers = {
        'RF': [0.2, -0.11, -0.2],  # Right Front
        'LH': [-0.2, 0.11, -0.2],  # Left Hind
        'LF': [0.2, 0.11, -0.2],   # Left Front
        'RH': [-0.2, -0.11, -0.2]  # Right Hind
    }

    initial_width = 0.1  # Radius along x-axis (horizontal stretch)
    initial_height = 0.01  # Radius along z-axis (vertical stretch)

    # Only set up sliders if using GUI
    if use_gui:
        width_slider = pb.addUserDebugParameter("Width", 0.01, 0.5, initial_width)
        height_slider = pb.addUserDebugParameter("Height", 0.01, 0.1, initial_height)
        angle_step_slider = pb.addUserDebugParameter("Angle Step", 0.01, 1.0, 0.05)

    angle = 0.0

    try:
        while True:
            # Reset robot state if "R" key is pressed
            if controls.is_reset_key_pressed():
                for i in range(num_simulations):
                    reset_robot(robots[i], i)

            # Determine parameters based on GUI or default
            if use_gui:
                angle_step = pb.readUserDebugParameter(angle_step_slider)
                x_radius = pb.readUserDebugParameter(width_slider)
                z_radius = pb.readUserDebugParameter(height_slider)
            else:
                angle_step = 0.05
                x_radius = 0.1
                z_radius = 0.01

            angle += angle_step
            if angle > 2 * math.pi:
                angle -= 2 * math.pi

            for robot_idx, qdrp in enumerate(robots):
                # Generate trajectory points for each leg
                RF_x, RF_y, RF_z = generate_oval_trajectory(oval_centers['RF'], x_radius, z_radius, angle)
                LH_x, LH_y, LH_z = generate_oval_trajectory(oval_centers['LH'], x_radius, z_radius, angle)
                LF_x, LF_y, LF_z = generate_oval_trajectory(oval_centers['LF'], x_radius, z_radius, angle + math.pi)
                RH_x, RH_y, RH_z = generate_oval_trajectory(oval_centers['RH'], x_radius, z_radius, angle + math.pi)

                # Target positions for legs
                targetPositionRF = np.array([0.2, -0.11, -0.2])
                targetPositionRH = np.array([-0.2, -0.11, -0.2])
                targetPositionLF = np.array([0.2, 0.11, -0.2])
                targetPositionLH = np.array([-0.2, 0.11, -0.2])

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

                # Log coordinates for RF leg
                coords = all_coordinates[robot_idx]
                coords['x_coords'].append(RF_x)
                coords['y_coords'].append(RF_y)
                coords['z_coords'].append(RF_z)

                # Keep only the last max_points
                if len(coords['x_coords']) > max_points:
                    coords['x_coords'] = coords['x_coords'][-max_points:]
                    coords['y_coords'] = coords['y_coords'][-max_points:]
                    coords['z_coords'] = coords['z_coords'][-max_points:]

            time.sleep(1/240.)  # Maintain simulation speed

        return all_coordinates

    except KeyboardInterrupt:
        print("Simulation stopped by user.")
        return all_coordinates
    finally:
        pb.disconnect()

def plot_trajectories(all_coordinates, num_simulations, max_points=500):
    """
    Plot the trajectories of the robots in 3D.
    """
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    lines = []
    points = []

    # Initialize plot elements for each robot
    for robot_id in range(num_simulations):
        line, = ax.plot([], [], [], label=f"Robot {robot_id} RF", lw=1)  # 3D trajectory line
        point, = ax.plot([], [], [], 'ro', markersize=2.5)  # Red dot for the latest point
        lines.append(line)
        points.append(point)

    def init():
        # Set axis labels and title
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title("3D Leg Trajectories")

        # Set reasonable axis limits
        ax.set_xlim(0.0, 0.4)
        ax.set_ylim(-0.5, 0.5)
        ax.set_zlim(-0.4, 0.0)

        ax.legend()
        return lines + points

    def update(frame):
        for robot_id in range(num_simulations):
            x_data = all_coordinates[robot_id]['x_coords']
            y_data = all_coordinates[robot_id]['y_coords']
            z_data = all_coordinates[robot_id]['z_coords']

            if x_data and y_data and z_data:  # Check if all lists have data
                lines[robot_id].set_data_3d(x_data, y_data, z_data)  # Update 3D trajectory

                # Ensure the last point is a sequence
                last_x = [x_data[-1]] if not isinstance(x_data[-1], (list, np.ndarray)) else x_data[-1]
                last_y = [y_data[-1]] if not isinstance(y_data[-1], (list, np.ndarray)) else y_data[-1]
                last_z = [z_data[-1]] if not isinstance(z_data[-1], (list, np.ndarray)) else z_data[-1]

                points[robot_id].set_data_3d(last_x, last_y, last_z)  # Update red dot
            else:
                lines[robot_id].set_data_3d([], [], [])
                points[robot_id].set_data_3d([], [], [])
        return lines + points

    # Create the animation
    ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=50)
    plt.show()

def main():
    num_simulations = 3 # Number of sequential simulations
    use_gui = True  # Set to True to show GUI, False for headless

    # Run simulations and get coordinates
    all_coordinates = run_sequential_simulation(num_simulations, use_gui)

    # Plot the trajectories
    plot_trajectories(all_coordinates, num_simulations)

if __name__ == "__main__":
    main()
