This program is an interactive 3D simulation of a three-joint robotic arm built using OpenGL and GLUT, designed to demonstrate principles of robotics, physics, 
and user interaction. The arm consists of three segments with lengths that can dynamically expand or contract based on a temperature range of -250°C to 250°C, 
controlled by the 'T' (increase by 5°C) and 'Y' (decrease by 5°C) keys, with a maximum length expansion of 0.0615% at 250°C. Users can manipulate the arm's target 
position using mouse motion, triggering smooth inverse kinematics-based movements to reach the desired point. Additionally, the simulation incorporates a customizable 
gravity vector, adjustable via the 'W' (up), 'S' (down), 'A' (left), and 'D' (right) keys, each altering the gravity direction by 0.2 units, affecting the base joint's 
rotation. The interface displays real-time data including joint angles (theta1, theta2, theta3), temperature, gravity vector, and a list of keyboard controls, rendered 
in white text on a black background. The arm is visualized with yellow cylinders for segments and orange spheres for joints, while a red sphere marks the target, 
providing a clear and engaging way to explore robotic motion under varying physical conditions.
