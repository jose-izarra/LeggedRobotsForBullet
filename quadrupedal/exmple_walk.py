import time
import numpy as np
from robot import Quadrupedal
import math


def generate_oval_trajectory(center, x_radius, z_radius, angle):
    """
    Generate a point on an oval trajectory.

    :param center: Center point of the oval [x, z]
    :param x_radius: Radius along x-axis
    :param z_radius: Radius along z-axis
    :param angle: Current angle (in radians)
    :return: [x, z] coordinates of the point
    """
    x = center[0] + x_radius * math.cos(-angle)
    z = center[1] + z_radius * math.sin(-angle)
    return x, z

def generate_half_oval_trajectory(center, x_radius, z_radius, angle):
    """
    Generate a point on a half-oval trajectory.

    :param center: Center point of the oval [x, z]
    :param x_radius: Radius along x-axis
    :param z_radius: Radius along z-axis
    :param angle: Current angle (in radians)
    :return: [x, z] coordinates of the point
    """
    ...


if __name__ == "__main__":
    # Initial target positions
    targetPositionRF = np.array([0.2, -0.11, -0.2])  # Right Front
    targetPositionRH = np.array([-0.2, -0.11, -0.2])  # Right Hind
    targetPositionLF = np.array([0.2, 0.11, -0.2])    # Left Front
    targetPositionLH = np.array([-0.2, 0.11, -0.2])   # Left Hind

    # Quadrupedal robot initialization
    qdrp = Quadrupedal(timeStep=1./240., initialCoMheight=0.3,
                       startPosition=[0, 0, 0.55], startOrientation=[0., 0., 0.],
                       maxForce=12, robotPATH="urdf/quadrupedal.urdf")

    # Oval trajectory parameters
    oval_center_f = [0.2, -0.2]  # Center of the oval path (front legs)
    oval_center_b = [-0.2, -0.2]  # Center of the oval path (back legs)
    x_radius = 0.1  # Radius along x-axis (horizontal stretch)
    z_radius = 0.01  # Radius along z-axis (vertical stretch)

    # Angle increment for smoother movement
    angle_step = 0.06
    angle = 0.0

    try:
        while True:
            # Update angles
            angle += angle_step
            if angle > 2 * math.pi:
                angle -= 2 * math.pi

            # Diagonal Pair 1: Front Right (RF) and Left Hind (LH)
            RF_x, RF_z = generate_oval_trajectory(oval_center_f, x_radius, z_radius, angle)
            LH_x, LH_z = generate_oval_trajectory(oval_center_b, x_radius, z_radius, angle)

            targetPositionRF[0], targetPositionRF[2] = RF_x, RF_z
            targetPositionLH[0], targetPositionLH[2] = LH_x, LH_z

            RFjointPositions = qdrp.inverseKinematics(targetPositionRF, targetLeg=qdrp.legRF)
            LHjointPositions = qdrp.inverseKinematics(targetPositionLH, targetLeg=qdrp.legLH)

            qdrp.legRF.setJointPositions(RFjointPositions)
            qdrp.legLH.setJointPositions(LHjointPositions)

            # Diagonal Pair 2: Front Left (LF) and Right Hind (RH) (phase-offset by π)
            LF_x, LF_z = generate_oval_trajectory(oval_center_f, x_radius, z_radius, angle + math.pi)
            RH_x, RH_z = generate_oval_trajectory(oval_center_b, x_radius, z_radius, angle + math.pi)

            targetPositionLF[0], targetPositionLF[2] = LF_x, LF_z
            targetPositionRH[0], targetPositionRH[2] = RH_x, RH_z

            LFjointPositions = qdrp.inverseKinematics(targetPositionLF, targetLeg=qdrp.legLF)
            RHjointPositions = qdrp.inverseKinematics(targetPositionRH, targetLeg=qdrp.legRH)

            qdrp.legLF.setJointPositions(LFjointPositions)
            qdrp.legRH.setJointPositions(RHjointPositions)

            # Advance simulation
            qdrp.oneStep()

    except KeyboardInterrupt:
        print("Simulation stopped by user.")
