#include <GL/glut.h>
#include <cmath>
#include <iostream>

// Arm parameters
const float L1 = 1.0f; // Length of first segment
const float L2 = 0.8f; // Length of second segment
float theta1 = 0.0f;   // Angle of first joint (radians)
float theta2 = 0.0f;   // Angle of second joint (radians)
float targetX = 1.5f;  // Target position X
float targetY = 0.5f;  // Target position Y
float targetZ = 0.0f;  // Target position Z (for simplicity)

// Forward Kinematics: Calculate end-effector position from joint angles
void forwardKinematics(float& x, float& y, float& z) {
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
    z = 0.0f; // 2D plane for simplicity, extend to 3D with more joints
}

// Inverse Kinematics: Calculate joint angles from target position
void inverseKinematics(float x, float y) {
    // Distance to target
    float r = sqrt(x * x + y * y);
    if (r > L1 + L2) {
        std::cout << "Target unreachable!" << std::endl;
        return;
    }

    // Cosine law for theta2
    float cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    theta2 = acos(cosTheta2);

    // Theta1 using atan2 and adjusted for theta2
    float k1 = L1 + L2 * cos(theta2);
    float k2 = L2 * sin(theta2);
    theta1 = atan2(y, x) - atan2(k2, k1);
}

// OpenGL display function
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // Calculate end-effector position
    float endX, endY, endZ;
    forwardKinematics(endX, endY, endZ);

    // Draw arm
    glBegin(GL_LINES);
    glColor3f(1.0, 1.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);              // Base
    glVertex3f(L1 * cos(theta1), L1 * sin(theta1), 0.0); // Joint 1

    glVertex3f(L1 * cos(theta1), L1 * sin(theta1), 0.0); // Joint 1
    glVertex3f(endX, endY, endZ);                        // End-effector
    glEnd();

    // Draw target
    glPointSize(10.0);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(targetX, targetY, targetZ);
    glEnd();

    glutSwapBuffers();
}

// Keyboard controls for target position
void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case 'w': targetY += 0.1f; break; // Move target up
    case 's': targetY -= 0.1f; break; // Move target down
    case 'a': targetX -= 0.1f; break; // Move target left
    case 'd': targetX += 0.1f; break; // Move target right
    case 'q': exit(0); break;         // Exit
    }
    inverseKinematics(targetX, targetY); // Update angles
    glutPostRedisplay();
}

// Initialize OpenGL
void init() {
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 1.0, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {
    // Initial inverse kinematics
    inverseKinematics(targetX, targetY);

    // GLUT setup
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 800);
    glutCreateWindow("Robotic Arm in Space - Kinematics Demo");

    init();
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutMainLoop();

    return 0;
}