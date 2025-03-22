#include <GL/glut.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const float BASE_L1 = 1.0f;
const float BASE_L2 = 0.8f;
const float BASE_L3 = 0.6f;
float L1 = BASE_L1;
float L2 = BASE_L2;
float L3 = BASE_L3;
float theta1 = 0.0f;
float theta2 = 0.0f;
float theta3 = 0.0f;
float currentTheta1 = 0.0f;
float currentTheta2 = 0.0f;
float currentTheta3 = 0.0f;
float targetX = 1.5f;
float targetY = 0.5f;
float targetZ = 0.0f;
float temperature = 0.0f;
float gravityX = 0.1f;
float gravityY = 0.0f;
std::chrono::steady_clock::time_point startTime;
bool transitioning = false;

template <typename T>
std::string to_string(T value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

void updateLengths() {
    float expansion = 0.0615f * (temperature / 250.0f);
    L1 = BASE_L1 * (1.0f + expansion);
    L2 = BASE_L2 * (1.0f + expansion);
    L3 = BASE_L3 * (1.0f + expansion);
}

void forwardKinematics(float& x, float& y, float& z) {
    float x1 = L1 * cos(currentTheta1);
    float y1 = L1 * sin(currentTheta1);
    float x2 = x1 + L2 * cos(currentTheta1 + currentTheta2);
    float y2 = y1 + L2 * sin(currentTheta1 + currentTheta2);
    x = x2 + L3 * cos(currentTheta1 + currentTheta2 + currentTheta3);
    y = y2 + L3 * sin(currentTheta1 + currentTheta2 + currentTheta3);
    z = 0.0f;
}

void inverseKinematics(float x, float y) {
    updateLengths();

    float dx = x - L3 * cos(theta1 + theta2 + theta3);
    float dy = y - L3 * sin(theta1 + theta2 + theta3);

    float r = sqrt(dx * dx + dy * dy);
    if (r > L1 + L2) return;

    float cosTheta2 = (dx * dx + dy * dy - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    cosTheta2 = std::max(-1.0f, std::min(1.0f, cosTheta2));
    theta2 = acos(cosTheta2);

    float k1 = L1 + L2 * cos(theta2);
    float k2 = L2 * sin(theta2);
    theta1 = atan2(dy, dx) - atan2(k2, k1);

    float endX = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    float endY = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
    theta3 = atan2(y - endY, x - endX) - (theta1 + theta2);
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
    float joint1X = L1 * cos(currentTheta1);
    float joint1Y = L1 * sin(currentTheta1);
    float joint2X = joint1X + L2 * cos(currentTheta1 + currentTheta2);
    float joint2Y = joint1Y + L2 * sin(currentTheta1 + currentTheta2);

    drawCylinder(0.0, 0.0, 0.0, joint1X, joint1Y, 0.0);
    drawCylinder(joint1X, joint1Y, 0.0, joint2X, joint2Y, 0.0);
    drawCylinder(joint2X, joint2Y, 0.0, endX, endY, endZ);

    glColor3f(1.0, 0.5, 0.0);
    glPushMatrix();
    glTranslatef(0.0, 0.0, 0.0);
    glutSolidSphere(0.15, 20, 20);
    glTranslatef(joint1X, joint1Y, 0.0);
    glutSolidSphere(0.15, 20, 20);
    glTranslatef(joint2X - joint1X, joint2Y - joint1Y, 0.0);
    glutSolidSphere(0.15, 20, 20);
    glTranslatef(endX - joint2X, endY - joint2Y, endZ);
    glutSolidSphere(0.15, 20, 20);
    glPopMatrix();

    glColor3f(1.0, 0.0, 0.0);
    glPushMatrix();
    glTranslatef(targetX, targetY, targetZ);
    glutSolidSphere(0.1, 20, 20);
    glPopMatrix();

    glColor3f(1.0, 1.0, 1.0);
    float startX = -1.9f;
    float startY = 1.9f;
    std::string info[] = {
        "Theta1: " + to_string(currentTheta1),
        "Theta2: " + to_string(currentTheta2),
        "Theta3: " + to_string(currentTheta3),
        "Temp: " + to_string(temperature) + "°C",
        "Gravity: (" + to_string(gravityX) + ", " + to_string(gravityY) + ")",
        "Controls:",
        "W: Gravity Up (+0.2)",
        "S: Gravity Down (-0.2)",
        "A: Gravity Left (-0.2)",
        "D: Gravity Right (+0.2)",
        "T: Temp +5°C",
        "Y: Temp -5°C"
    };

    for (int i = 0; i < sizeof(info) / sizeof(info[0]); ++i) {
        glRasterPos2f(startX, startY - i * 0.1f);  // Adjust line spacing with 0.1f
        for (char c : info[i]) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
    }

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

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case 'w':
        gravityY += 0.2f;
        break;
    case 's':
        gravityY -= 0.2f;
        break;
    case 'a':
        gravityX -= 0.2f;
        break;
    case 'd':
        gravityX += 0.2f;
        break;
    case 't':
        temperature = std::min(120.0f, temperature + 5.0f);
        break;
    case 'y':
        temperature = std::max(-120.0f, temperature - 5.0f);
        break;
    }
    updateLengths();
    glutPostRedisplay();
}

void update(int value) {
    if (transitioning) {
        float elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - startTime).count();
        float t = fmin(elapsed / 2.0f, 1.0f);

        currentTheta1 = (1 - t) * currentTheta1 + t * theta1;
        currentTheta2 = (1 - t) * currentTheta2 + t * theta2;
        currentTheta3 = (1 - t) * currentTheta3 + t * theta3;

        if (t >= 1.0f) transitioning = false;
    }

    float endX, endY, endZ;
    forwardKinematics(endX, endY, endZ);
    float totalMass = 1.0f;
    float gravityTorqueX = gravityX * totalMass * endX;
    float gravityTorqueY = gravityY * totalMass * endY;
    float totalTorque = gravityTorqueX + gravityTorqueY;
    currentTheta1 -= totalTorque * 0.01f;

    glutPostRedisplay();
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
    glutCreateWindow("Smooth Robotic Arm with Gravity and Temperature");
    init();
    glutDisplayFunc(display);
    glutMotionFunc(mouseMotion);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(16, update, 0);
    glutMainLoop();
    return 0;
}