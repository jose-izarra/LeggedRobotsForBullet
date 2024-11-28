import pybullet as pb
import pybullet_data
from robot import Quadrupedal

def main():
    # Connect to PyBullet GUI
    pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.8)

    # Create multiple robot instances
    robot1 = Quadrupedal(
        timeStep=1./240.,
        initialCoMheight=0.3,
        robotPATH="../urdf/quadrupedal.urdf",
        startPosition=[0, 0, 0.55],
        startOrientation=[0., 0., 0.],
        maxForce=12
    )

    robot2 = Quadrupedal(
        timeStep=1./240.,
        initialCoMheight=0.3,
        robotPATH="../urdf/quadrupedal.urdf",
        startPosition=[1, 0, 0.55],  # Offset position for the second robot
        startOrientation=[0., 0., 0.],
        maxForce=12
    )

    try:
        while True:
            # Run the simulation for both robots
            robot1.oneStep()
            robot2.oneStep()

    except KeyboardInterrupt:
        print("Simulation stopped by user.")

    finally:
        pb.disconnect()

if __name__ == "__main__":
    main()
