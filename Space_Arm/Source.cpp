#include <GL/glut.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const float L1 = 1.0f;
const float L2 = 0.8f;
float theta1 = 0.0f;
float theta2 = 0.0f;
float currentTheta1 = 0.0f;
float currentTheta2 = 0.0f;
float targetX = 1.5f;
float targetY = 0.5f;
float targetZ = 0.0f;
std::chrono::steady_clock::time_point startTime;
bool transitioning = false;

// Custom to_string function
template <typename T>
std::string to_string(T value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

void forwardKinematics(float& x, float& y, float& z) {
    x = L1 * cos(currentTheta1) + L2 * cos(currentTheta1 + currentTheta2);
    y = L1 * sin(currentTheta1) + L2 * sin(currentTheta1 + currentTheta2);
    z = 0.0f;
}

void inverseKinematics(float x, float y) {
    float r = sqrt(x * x + y * y);
    if (r > L1 + L2) return;

    float cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    theta2 = acos(cosTheta2);
    float k1 = L1 + L2 * cos(theta2);
    float k2 = L2 * sin(theta2);
    theta1 = atan2(y, x) - atan2(k2, k1);
}

void drawCylinder(float x1, float y1, float z1, float x2, float y2, float z2) {
    GLUquadric* quad = gluNewQuadric();
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    float length = sqrt(dx * dx + dy * dy + dz * dz);

    glPushMatrix();
    glTranslatef(x1, y1, z1);

    if (length > 0.0001f) {
        float angle = acos(dz / length) * 180.0 / M_PI;
        float axisX = -dy;
        float axisY = dx;
        float axisLength = sqrt(axisX * axisX + axisY * axisY);

        if (axisLength > 0.0001f) {
            axisX /= axisLength;
            axisY /= axisLength;
            glRotatef(angle, axisX, axisY, 0.0f);
        }
    }

    glColor3f(1.0, 1.0, 0.0);
    gluCylinder(quad, 0.1, 0.1, length, 20, 20);

    glPopMatrix();
    gluDeleteQuadric(quad);
}


void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    float endX, endY, endZ;
    forwardKinematics(endX, endY, endZ);
    float jointX = L1 * cos(currentTheta1);
    float jointY = L1 * sin(currentTheta1);

    drawCylinder(0.0, 0.0, 0.0, jointX, jointY, 0.0);
    drawCylinder(jointX, jointY, 0.0, endX, endY, endZ);

    glColor3f(1.0, 0.5, 0.0);
    glPushMatrix();
    glTranslatef(0.0, 0.0, 0.0);
    glutSolidSphere(0.15, 20, 20);
    glTranslatef(jointX, jointY, 0.0);
    glutSolidSphere(0.15, 20, 20);
    glTranslatef(endX - jointX, endY - jointY, endZ);
    glutSolidSphere(0.15, 20, 20);
    glPopMatrix();

    glColor3f(1.0, 0.0, 0.0);
    glPushMatrix();
    glTranslatef(targetX, targetY, targetZ);
    glutSolidSphere(0.1, 20, 20);
    glPopMatrix();

    glColor3f(1.0, 1.0, 1.0);
    glRasterPos2f(-1.9f, 1.9f);

    std::string angles = "Theta1: " + to_string(currentTheta1) + " | Theta2: " + to_string(currentTheta2);
    for (char c : angles) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);

    glutSwapBuffers();
}

void mouseMotion(int x, int y) {
    int windowWidth = glutGet(GLUT_WINDOW_WIDTH);
    int windowHeight = glutGet(GLUT_WINDOW_HEIGHT);

    targetX = (float)(x - windowWidth / 2) / (windowWidth / 4);
    targetY = -(float)(y - windowHeight / 2) / (windowHeight / 4);

    inverseKinematics(targetX, targetY);
    startTime = std::chrono::steady_clock::now();
    transitioning = true;
}

void update(int value) {
    if (transitioning) {
        float elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - startTime).count();
        float t = fmin(elapsed / 2.0f, 1.0f);

        currentTheta1 = currentTheta1 + t * (theta1 - currentTheta1);
        currentTheta2 = currentTheta2 + t * (theta2 - currentTheta2);

        if (t >= 1.0f) transitioning = false;
        glutPostRedisplay();
    }
    glutTimerFunc(16, update, 0);
}

void init() {
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    gluPerspective(45.0, 1.0, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 800);
    glutCreateWindow("Smooth Robotic Arm");
    init();
    glutDisplayFunc(display);
    glutMotionFunc(mouseMotion);
    glutTimerFunc(16, update, 0);
    glutMainLoop();
    return 0;
}
