#include <GL/glut.h>
#include <cmath>
#include <iostream>

// Define M_PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Arm parameters
const float L1 = 1.0f; // Length of first segment
const float L2 = 0.8f; // Length of second segment
float theta1 = 0.0f;   // Angle of first joint (radians)
float theta2 = 0.0f;   // Angle of second joint (radians)
float targetX = 1.5f;  // Target position X
float targetY = 0.5f;  // Target position Y
float targetZ = 0.0f;  // Target position Z

// Forward Kinematics
void forwardKinematics(float& x, float& y, float& z) {
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
    z = 0.0f;
}

// Inverse Kinematics
void inverseKinematics(float x, float y) {
    float r = sqrt(x * x + y * y);
    if (r > L1 + L2) return;

    float cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    theta2 = acos(cosTheta2);
    float k1 = L1 + L2 * cos(theta2);
    float k2 = L2 * sin(theta2);
    theta1 = atan2(y, x) - atan2(k2, k1);
}

// Draw a cylinder between two points with proper orientation
void drawCylinder(float x1, float y1, float z1, float x2, float y2, float z2) {
    GLUquadric* quad = gluNewQuadric();
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    float length = sqrt(dx * dx + dy * dy + dz * dz);

    glPushMatrix();
    glTranslatef(x1, y1, z1);

    float angle = acos(dz / length) * 180 / M_PI;
    float axisX = -dy;
    float axisY = dx;
    glRotatef(angle, axisX, axisY, 0.0f);

    glColor3f(1.0, 1.0, 0.0); // Yellow color
    gluCylinder(quad, 0.1, 0.1, length, 20, 20);
    glPopMatrix();

    gluDeleteQuadric(quad);
}

// Display function
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    float endX, endY, endZ;
    forwardKinematics(endX, endY, endZ);
    float jointX = L1 * cos(theta1);
    float jointY = L1 * sin(theta1);

    // Draw arm segments as yellow cylinders
    drawCylinder(0.0, 0.0, 0.0, jointX, jointY, 0.0);
    drawCylinder(jointX, jointY, 0.0, endX, endY, endZ);

    // Draw joints as orange spheres
    glColor3f(1.0, 0.5, 0.0); // Orange color
    glPushMatrix();
    glTranslatef(0.0, 0.0, 0.0);
    glutSolidSphere(0.15, 20, 20);
    glTranslatef(jointX, jointY, 0.0);
    glutSolidSphere(0.15, 20, 20);
    glTranslatef(endX - jointX, endY - jointY, endZ);
    glutSolidSphere(0.15, 20, 20);
    glPopMatrix();

    // Draw target as red sphere
    glColor3f(1.0, 0.0, 0.0);
    glPushMatrix();
    glTranslatef(targetX, targetY, targetZ);
    glutSolidSphere(0.1, 20, 20);
    glPopMatrix();

    glutSwapBuffers();
}

// Mouse motion handler
void mouseMotion(int x, int y) {
    int windowWidth = glutGet(GLUT_WINDOW_WIDTH);
    int windowHeight = glutGet(GLUT_WINDOW_HEIGHT);

    targetX = (float)(x - windowWidth / 2) / (windowWidth / 4);
    targetY = -(float)(y - windowHeight / 2) / (windowHeight / 4);

    inverseKinematics(targetX, targetY);
    glutPostRedisplay();
}

// Initialize OpenGL
void init() {
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat lightPos[] = { 0.0, 0.0, 5.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glEnable(GL_COLOR_MATERIAL);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 1.0, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {
    inverseKinematics(targetX, targetY);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 800);
    glutCreateWindow("Robotic Arm with Cylinders - Fixed Orientation");

    init();
    glutDisplayFunc(display);
    glutMotionFunc(mouseMotion);
    glutMainLoop();

    return 0;
}
