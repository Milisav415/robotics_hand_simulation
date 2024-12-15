# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton
from PyQt5.QtCore import QThread, pyqtSignal
import pybullet as p
import pybullet_data
import time


class SimulationThread(QThread):
    joint_positions_signal = pyqtSignal(list)  # Signal to emit joint positions
    camera_image_signal = pyqtSignal(np.ndarray)  # New signal for camera images

    def __init__(self):
        super().__init__()
        self.endEffectorIndex = None
        self.robotId = None
        self.target_pos = [0.4, 0, 0.5]  # Initial target position
        self.target_ori = p.getQuaternionFromEuler([0, 0, 0])  # No rotation

    def run(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        self.robotId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
        self.endEffectorIndex = 6  # Kuka IIWA

        # Adding a simple object (a box) to pick up
        self.objectUid = p.loadURDF("cube_small.urdf", [0.5, 0, 0.5])

        p.setGravity(0, 0, -10)
        self.pickObject()

        while True:
            jointPoses = p.calculateInverseKinematics(self.robotId, self.endEffectorIndex, self.target_pos, self.target_ori)
            for i, jointPose in enumerate(jointPoses):
                p.setJointMotorControl2(self.robotId, i, p.POSITION_CONTROL, jointPose)

            # Emit current joint positions
            jointPositions = [p.getJointState(self.robotId, i)[0] for i in range(p.getNumJoints(self.robotId))]
            self.joint_positions_signal.emit(jointPositions)

            p.stepSimulation()
            self.updateCamera()
            time.sleep(1. / 240.)

    def pickObject(self):
        # Move robot to pick the object (simplified; actual implementation would use IK for precise control)
        # Here you would calculate the joint positions for picking up the object and use them
        # For demonstration, we skip directly to 'attaching' the object

        # Create a constraint to attach the object to the end effector (gripper simulation)
        cid = p.createConstraint(self.robotId, self.endEffectorIndex, self.objectUid, -1, p.JOINT_FIXED, [0, 0, 0],
                                 [0, 0, 0.2], [0, 0, 0])

    def updateCamera(self):
        # Assume the camera is mounted on the end effector; we use the end effector's position and orientation
        link_state = p.getLinkState(self.robotId, self.endEffectorIndex, computeForwardKinematics=True)
        cam_pos = link_state[0]
        cam_ori = link_state[1]
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cam_pos, distance=0.1, yaw=0, pitch=-90, roll=0,
                                                          upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(60, 640 / 480, 0.01, 100)
        img = p.getCameraImage(640, 480, view_matrix, proj_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)

    def update_target_position(self, pos):
        self.target_pos = pos


class TcpInputWindow(QWidget):
    new_position_signal = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.joint_labels = []
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        self.position_input = QLineEdit(self)
        self.position_input.setPlaceholderText("Enter TCP position as x,y,z")
        layout.addWidget(self.position_input)

        self.set_position_button = QPushButton('Set TCP Position', self)
        self.set_position_button.clicked.connect(self.on_set_position_clicked)
        layout.addWidget(self.set_position_button)

        # Add labels for displaying joint positions
        for i in range(7):  # Assuming 7 joints for Kuka IIWA
            label = QLabel(f"Joint {i+1} Position: N/A")
            self.joint_labels.append(label)
            layout.addWidget(label)

        self.setLayout(layout)
        self.setWindowTitle('TCP Position Input')

    def on_set_position_clicked(self):
        pos_text = self.position_input.text()
        try:
            pos = list(map(float, pos_text.split(',')))
            self.new_position_signal.emit(pos)
        except ValueError:
            print("Invalid input. Please enter the position as 'x,y,z'.")

    def update_joint_positions(self, positions):
        for i, position in enumerate(positions):
            self.joint_labels[i].setText(f"Joint {i+1} Position: {position:.2f}")


if __name__ == '__main__':
    app = QApplication(sys.argv)

    simulation_thread = SimulationThread()

    tcp_input_window = TcpInputWindow()
    tcp_input_window.new_position_signal.connect(simulation_thread.update_target_position)
    simulation_thread.joint_positions_signal.connect(tcp_input_window.update_joint_positions)

    simulation_thread.start()
    tcp_input_window.show()

    sys.exit(app.exec_())

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
