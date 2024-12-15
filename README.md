# Robotic Hand Simulation with Real-Time Control  

This project simulates a robotic hand using the PyBullet physics engine and provides a graphical interface for real-time control and visualization. The system models a KUKA IIWA robotic arm, enabling users to interact with it through a GUI and set the tool center point (TCP) positions dynamically.  

## Key Features  

- **Real-Time Simulation**  
  - Uses PyBullet for dynamic simulation of the robotic hand and environment.  
  - Step-by-step simulation with real-time joint position updates.  

- **Graphical User Interface (GUI)**  
  - Built with PyQt5 for easy input of TCP positions.  
  - Displays live joint angles for better monitoring and interaction.  

- **Interactive Control**  
  - Allows users to set the target position of the robotic arm via a simple GUI input.  
  - Implements inverse kinematics (IK) for precise motion of the end effector.  

- **Object Interaction**  
  - Simulates object manipulation by adding constraints to "pick and place" objects.  

- **Camera Integration**  
  - A virtual camera mounted on the robotic arm provides real-time images of the workspace.

## Technologies Used  

- **Programming Language:** Python  
- **Physics Engine:** PyBullet  
- **GUI Framework:** PyQt5  
- **Libraries:** NumPy  

## How to Run  

1. Ensure Python 3.x is installed.  
2. Install dependencies:  
   ```bash
   pip install pybullet pyqt5 numpy
