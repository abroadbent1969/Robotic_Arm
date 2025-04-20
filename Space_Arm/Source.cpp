#include <GL/glut.h>
#include <cmath>
#include <iostream>
#include <chrono>
#include <sstream>
#include <vector>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Arm parameters
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
float torqueFactor = 0.01f;
std::chrono::steady_clock::time_point startTime;
bool transitioning = false;

// Claw and cylinder variables
float clawAngle = 15.9f;
const float MAX_CLAW_ANGLE = 45.0f;
bool clawHolding = false;
float cylinderX = 1.5f;
float cylinderY = 1.5f;
float cylinderZ = 0.0f;
float cylinderVelX = 0.0f;
float cylinderVelY = 0.0f;
float cylinderVelZ = 0.0f;
const float CYLINDER_RADIUS = 0.11f;
const float CYLINDER_HEIGHT = 0.5f;

// Physics parameters
float GRAVITY = 0.0f; // Initial gravity set to 0
const float DT = 0.016f; // Time step (16ms for 60 FPS)
float mass1 = 1.0f; // Mass of link 1 (kg)
float mass2 = 0.8f; // Mass of link 2
float mass3 = 0.6f; // Mass of link 3
float I1 = 0.2f * mass1 * L1 * L1; // Moment of inertia for link 1
float I2 = 0.2f * mass2 * L2 * L2; // Moment of inertia for link 2
float I3 = 0.2f * mass3 * L3 * L3; // Moment of inertia for link 3
float omega1 = 0.0f; // Angular velocity for joint 1 (rad/s)
float omega2 = 0.0f; // Angular velocity for joint 2
float omega3 = 0.0f; // Angular velocity for joint 3
const float JOINT_FRICTION = 2.5f; // Viscous friction coefficient
const float JOINT_LIMIT_MIN = -M_PI; // Joint angle limits
const float JOINT_LIMIT_MAX = M_PI;
float cylinderMass = 1.5f; // Cylinder mass (kg)
const float TORQUE_SCALE = 3.0f; // Scale torques for stiffer motion
const float COLLISION_DAMPING = 0.02f; // Damping factor for collision response

// PID Controller parameters for fluid movement
const float KP = 10.0f; // Proportional gain
const float KI = 0.1f;  // Integral gain
const float KD = 1.0f;  // Derivative gain
float integral1 = 0.0f, integral2 = 0.0f, integral3 = 0.0f;
float prevError1 = 0.0f, prevError2 = 0.0f, prevError3 = 0.0f;

// AABB structure for poly-to-poly collision
struct AABB {
    float minX, maxX, minY, maxY, minZ, maxZ;
    AABB(float _minX, float _maxX, float _minY, float _maxY, float _minZ, float _maxZ)
        : minX(_minX), maxX(_maxX), minY(_minY), maxY(_maxY), minZ(_minZ), maxZ(_maxZ) {
    }
};

void updateLengths() {
    float expansion = 0.0615f * (temperature / 250.0f);
    L1 = BASE_L1 * (1.0f + expansion);
    L2 = BASE_L2 * (1.0f + expansion);
    L3 = BASE_L3 * (1.0f + expansion);
    I1 = 0.2f * mass1 * L1 * L1;
    I2 = 0.2f * mass2 * L2 * L2;
    I3 = 0.2f * mass3 * L3 * L3;
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

// AABB collision detection
bool checkAABBCollision(const AABB& a, const AABB& b) {
    return (a.minX <= b.maxX && a.maxX >= b.minX) &&
        (a.minY <= b.maxY && a.maxY >= b.minY) &&
        (a.minZ <= b.maxZ && a.maxZ >= b.minZ);
}

// Create AABB for arm segment
AABB createSegmentAABB(float x1, float y1, float z1, float x2, float y2, float z2, float radius) {
    float minX = std::min(x1, x2) - radius;
    float maxX = std::max(x1, x2) + radius;
    float minY = std::min(y1, y2) - radius;
    float maxY = std::max(y1, y2) + radius;
    float minZ = std::min(z1, z2) - radius;
    float maxZ = std::max(z1, z2) + radius;
    return AABB(minX, maxX, minY, maxY, minZ, maxZ);
}

// Create AABB for cylinder
AABB createCylinderAABB() {
    return AABB(
        cylinderX - CYLINDER_RADIUS, cylinderX + CYLINDER_RADIUS,
        cylinderY - CYLINDER_RADIUS, cylinderY + CYLINDER_RADIUS,
        cylinderZ - CYLINDER_HEIGHT / 2, cylinderZ + CYLINDER_HEIGHT / 2
    );
}

// Check collision and compute push velocity
bool checkArmCollision(float x1, float y1, float z1, float x2, float y2, float z2, float radius, float& pushVelX, float& pushVelY) {
    AABB segment = createSegmentAABB(x1, y1, z1, x2, y2, z2, radius);
    AABB cylinder = createCylinderAABB();
    if (checkAABBCollision(segment, cylinder)) {
        // Compute segment velocity (approximate from endpoint movement)
        float segmentVelX = (x2 - x1) / DT;
        float segmentVelY = (y2 - y1) / DT;

        // Apply impulse to cylinder (simplified momentum transfer)
        pushVelX = segmentVelX * COLLISION_DAMPING * mass1 / cylinderMass;
        pushVelY = segmentVelY * COLLISION_DAMPING * mass1 / cylinderMass;
        return true;
    }
    pushVelX = pushVelY = 0.0f;
    return false;
}

void drawCylinder(float x1, float y1, float z1, float x2, float y2, float z2, float radius = 0.1f, bool isClaw = false, bool isObject = false) {
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

    if (isClaw) glColor3f(0.7, 0.7, 0.7);
    else if (isObject) glColor3f(0.0, 1.0, 0.0);
    else glColor3f(1.0, 1.0, 0.0);
    gluCylinder(quad, radius, radius, length, 5, 5);
    glPopMatrix();
    gluDeleteQuadric(quad);
}

void drawStationaryCylinder(float x, float y, float z) {
    GLUquadric* quad = gluNewQuadric();
    glPushMatrix();
    glTranslatef(x, y, z - CYLINDER_HEIGHT / 2);
    glColor3f(0.0, 1.0, 0.0);
    gluCylinder(quad, CYLINDER_RADIUS, CYLINDER_RADIUS, CYLINDER_HEIGHT, 10, 10);
    gluDisk(quad, 0, CYLINDER_RADIUS, 20, 5);
    glTranslatef(0, 0, CYLINDER_HEIGHT);
    gluDisk(quad, 0, CYLINDER_RADIUS, 20, 5);
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

    drawCylinder(0.0, 0.0, 0.0, joint1X, joint1Y, 0.0, 0.12f);
    drawCylinder(joint1X, joint1Y, 0.3, joint2X, joint2Y, 0.0, 0.1f);
    drawCylinder(joint2X, joint2Y, -0.3, endX, endY, endZ, 0.08f);

    float clawLength = 0.3f;
    float clawBaseX = endX;
    float clawBaseY = endY;
    float clawBaseZ = endZ;
    float clawAngleRad = clawAngle * M_PI / 180.0f;
    float defaultOpenAngle = 15.0f * M_PI / 180.0f;

    float clawLeftMidX = clawBaseX + clawLength * cos(currentTheta1 + currentTheta2 + currentTheta3 + defaultOpenAngle + clawAngleRad);
    float clawLeftMidY = clawBaseY + clawLength * sin(currentTheta1 + currentTheta2 + currentTheta3 + defaultOpenAngle + clawAngleRad);
    float clawLeftEndX = clawLeftMidX + clawLength * 0.7f * cos(currentTheta1 + currentTheta2 + currentTheta3 + defaultOpenAngle + clawAngleRad - 0.5f);
    float clawLeftEndY = clawLeftMidY + clawLength * 0.7f * sin(currentTheta1 + currentTheta2 + currentTheta3 + defaultOpenAngle + clawAngleRad - 0.5f);

    drawCylinder(clawBaseX, clawBaseY, clawBaseZ, clawLeftMidX, clawLeftMidY, clawBaseZ, 0.05f, true);
    drawCylinder(clawLeftMidX, clawLeftMidY, clawBaseZ, clawLeftEndX, clawLeftEndY, clawBaseZ, 0.04f, true);

    float clawRightMidX = clawBaseX + clawLength * cos(currentTheta1 + currentTheta2 + currentTheta3 - defaultOpenAngle - clawAngleRad);
    float clawRightMidY = clawBaseY + clawLength * sin(currentTheta1 + currentTheta2 + currentTheta3 - defaultOpenAngle - clawAngleRad);
    float clawRightEndX = clawRightMidX + clawLength * 0.7f * cos(currentTheta1 + currentTheta2 + currentTheta3 - defaultOpenAngle - clawAngleRad + 0.5f);
    float clawRightEndY = clawRightMidY + clawLength * 0.7f * sin(currentTheta1 + currentTheta2 + currentTheta3 - defaultOpenAngle - clawAngleRad + 0.5f);

    drawCylinder(clawBaseX, clawBaseY, clawBaseZ, clawRightMidX, clawRightMidY, clawBaseZ, 0.05f, true);
    drawCylinder(clawRightMidX, clawRightMidY, clawBaseZ, clawRightEndX, clawRightEndY, clawBaseZ, 0.04f, true);

    glColor3f(1.0, 0.5, 0.0);
    glPushMatrix();
    glTranslatef(0.0, 0.0, 0.0);
    glutSolidSphere(0.13, 20, 20);
    glTranslatef(joint1X, joint1Y, 0.0);
    glutSolidSphere(0.13, 20, 20);
    glTranslatef(joint2X - joint1X, joint2Y - joint1Y, 0.0);
    glutSolidSphere(0.13, 20, 20);
    glTranslatef(endX - joint2X, endY - joint2Y, endZ);
    glutSolidSphere(0.13, 20, 20);
    glPopMatrix();

    glColor3f(1.0, 0.0, 0.0);
    glPushMatrix();
    glTranslatef(targetX, targetY, targetZ);
    glutSolidSphere(0.13, 20, 20);
    glPopMatrix();

    drawStationaryCylinder(cylinderX, cylinderY, clawHolding ? cylinderZ : 0.0f);

    glColor3f(1.0, 1.0, 1.0);
    float startX = -1.95f;
    float startY = 1.9f;
    std::string info[19] = {
        "Theta1: " + std::to_string(currentTheta1) + " rad",
        "Theta2: " + std::to_string(currentTheta2) + " rad",
        "Theta3: " + std::to_string(currentTheta3) + " rad",
        "Omega1: " + std::to_string(omega1) + " rad/s",
        "Omega2: " + std::to_string(omega2) + " rad/s",
        "Omega3: " + std::to_string(omega3) + " rad/s",
        "Gravity: " + std::to_string(GRAVITY) + " m/s^2",
        "Temp: " + std::to_string(temperature) + "°C",
        "Claw Angle: " + std::to_string(clawAngle) + " deg",
        "Cylinder Held: " + std::string(clawHolding ? "Yes" : "No"),
        "Controls:",
        "T: Temp +5°C",
        "Y: Temp -5°C",
        "N: Torque +0.005",
        "M: Torque -0.005",
        "C: Close Claw",
        "V: Open Claw",
        "Q: Decrease Gravity",
        "W: Increase Gravity"
    };

    for (int i = 0; i < 19; ++i) {
        glRasterPos2f(startX, startY - i * 0.1f);
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
    case 't':
        temperature = std::min(120.0f, temperature + 5.0f);
        break;
    case 'y':
        temperature = std::max(-120.0f, temperature - 5.0f);
        break;
    case 'n':
        torqueFactor = std::min(0.1f, torqueFactor + 0.005f);
        break;
    case 'm':
        torqueFactor = std::max(0.0f, torqueFactor - 0.005f);
        break;
    case 'c':
        clawAngle = std::max(0.0f, clawAngle - 5.0f);
        break;
    case 'v':
        clawAngle = std::min(MAX_CLAW_ANGLE, clawAngle + 5.0f);
        break;
    case 'q':
        GRAVITY = std::max(0.0f, GRAVITY - 0.5f); // Decrease gravity
        break;
    case 'w':
        GRAVITY = std::min(9.81f, GRAVITY + 0.5f); // Increase gravity
        break;
    }
    updateLengths();
    glutPostRedisplay();
}

void computeJointTorques(float& torque1, float& torque2, float& torque3) {
    float com1X = 0.5f * L1 * cos(currentTheta1);
    float com1Y = 0.5f * L1 * sin(currentTheta1);
    float joint1X = L1 * cos(currentTheta1);
    float joint1Y = L1 * sin(currentTheta1);
    float com2X = joint1X + 0.5f * L2 * cos(currentTheta1 + currentTheta2);
    float com2Y = joint1Y + 0.5f * L2 * sin(currentTheta1 + currentTheta2);
    float joint2X = joint1X + L2 * cos(currentTheta1 + currentTheta2);
    float joint2Y = joint1Y + L2 * sin(currentTheta1 + currentTheta2);
    float com3X = joint2X + 0.5f * L3 * cos(currentTheta1 + currentTheta2 + currentTheta3);
    float com3Y = joint2Y + 0.5f * L3 * sin(currentTheta1 + currentTheta2 + currentTheta3);

    float F1y = -mass1 * GRAVITY;
    float F2y = -mass2 * GRAVITY;
    float F3y = -mass3 * GRAVITY;

    torque1 = (com1X * F1y) + (com2X * F2y) + (com3X * F3y);
    torque2 = (com2X - joint1X) * F2y + (com3X - joint1X) * F3y;
    torque3 = (com3X - joint2X) * F3y;

    torque1 *= TORQUE_SCALE;
    torque2 *= TORQUE_SCALE;
    torque3 *= TORQUE_SCALE;

    torque1 -= JOINT_FRICTION * omega1;
    torque2 -= JOINT_FRICTION * omega2;
    torque3 -= JOINT_FRICTION * omega3;
}

// PID controller for smooth joint movement
void applyPIDControl(float& torque1, float& torque2, float& torque3) {
    float error1 = theta1 - currentTheta1;
    float error2 = theta2 - currentTheta2;
    float error3 = theta3 - currentTheta3;

    integral1 += error1 * DT;
    integral2 += error2 * DT;
    integral3 += error3 * DT;

    float derivative1 = (error1 - prevError1) / DT;
    float derivative2 = (error2 - prevError2) / DT;
    float derivative3 = (error3 - prevError3) / DT;

    torque1 += KP * error1 + KI * integral1 + KD * derivative1;
    torque2 += KP * error2 + KI * integral2 + KD * derivative2;
    torque3 += KP * error3 + KI * integral3 + KD * derivative3;

    prevError1 = error1;
    prevError2 = error2;
    prevError3 = error3;
}

void update(int value) {
    if (transitioning) {
        float elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - startTime).count();
        float t = fmin(elapsed / 2.0f, 1.0f);

        float prevTheta1 = currentTheta1;
        float prevTheta2 = currentTheta2;
        float prevTheta3 = currentTheta3;

        // Apply PID control for smooth movement
        float torque1 = 0.0f, torque2 = 0.0f, torque3 = 0.0f;
        applyPIDControl(torque1, torque2, torque3);

        float alpha1 = torque1 / I1;
        float alpha2 = torque2 / I2;
        float alpha3 = torque3 / I3;

        omega1 += alpha1 * DT;
        omega2 += alpha2 * DT;
        omega3 += alpha3 * DT;

        currentTheta1 += omega1 * DT;
        currentTheta2 += omega2 * DT;
        currentTheta3 += omega3 * DT;

        float endX, endY, endZ;
        forwardKinematics(endX, endY, endZ);
        float joint1X = L1 * cos(currentTheta1);
        float joint1Y = L1 * sin(currentTheta1);
        float joint2X = joint1X + L2 * cos(currentTheta1 + currentTheta2);
        float joint2Y = joint1Y + L2 * sin(currentTheta1 + currentTheta2);

        bool armCollision = false;
        float pushVelX = 0.0f, pushVelY = 0.0f;
        armCollision |= checkArmCollision(0.0f, 0.0f, 0.0f, joint1X, joint1Y, 0.0f, 0.12f, pushVelX, pushVelY);
        armCollision |= checkArmCollision(joint1X, joint1Y, 0.3f, joint2X, joint2Y, 0.0f, 0.1f, pushVelX, pushVelY);
        armCollision |= checkArmCollision(joint2X, joint2Y, -0.3f, endX, endY, endZ, 0.08f, pushVelX, pushVelY);

        float clawLength = 0.3f;
        float clawBaseX = endX;
        float clawBaseY = endY;
        float clawBaseZ = endZ;
        float clawAngleRad = clawAngle * M_PI / 180.0f;
        float defaultOpenAngle = 15.0f * M_PI / 180.0f;

        float clawLeftMidX = clawBaseX + clawLength * cos(currentTheta1 + currentTheta2 + currentTheta3 + defaultOpenAngle + clawAngleRad);
        float clawLeftMidY = clawBaseY + clawLength * sin(currentTheta1 + currentTheta2 + currentTheta3 + defaultOpenAngle + clawAngleRad);
        float clawLeftEndX = clawLeftMidX + clawLength * 0.7f * cos(currentTheta1 + currentTheta2 + currentTheta3 + defaultOpenAngle + clawAngleRad - 0.5f);
        float clawLeftEndY = clawLeftMidY + clawLength * 0.7f * sin(currentTheta1 + currentTheta2 + currentTheta3 + defaultOpenAngle + clawAngleRad - 0.5f);

        float clawRightMidX = clawBaseX + clawLength * cos(currentTheta1 + currentTheta2 + currentTheta3 - defaultOpenAngle - clawAngleRad);
        float clawRightMidY = clawBaseY + clawLength * sin(currentTheta1 + currentTheta2 + currentTheta3 - defaultOpenAngle - clawAngleRad);
        float clawRightEndX = clawRightMidX + clawLength * 0.7f * cos(currentTheta1 + currentTheta2 + currentTheta3 - defaultOpenAngle - clawAngleRad + 0.5f);
        float clawRightEndY = clawRightMidY + clawLength * 0.7f * sin(currentTheta1 + currentTheta2 + currentTheta3 - defaultOpenAngle - clawAngleRad + 0.5f);

        bool clawCollision = false;
        clawCollision |= checkArmCollision(clawBaseX, clawBaseY, clawBaseZ, clawLeftMidX, clawLeftMidY, clawBaseZ, 0.05f, pushVelX, pushVelY);
        clawCollision |= checkArmCollision(clawLeftMidX, clawLeftMidY, clawBaseZ, clawLeftEndX, clawLeftEndY, clawBaseZ, 0.04f, pushVelX, pushVelY);
        clawCollision |= checkArmCollision(clawBaseX, clawBaseY, clawBaseZ, clawRightMidX, clawRightMidY, clawBaseZ, 0.05f, pushVelX, pushVelY);
        clawCollision |= checkArmCollision(clawRightMidX, clawRightMidY, clawBaseZ, clawRightEndX, clawRightEndY, clawBaseZ, 0.04f, pushVelX, pushVelY);

        // Update cylinder velocity if collision occurs
        if ((armCollision || clawCollision) && !clawHolding) {
            cylinderVelX += pushVelX;
            cylinderVelY += pushVelY;
            // Allow arm to continue moving (no angle reset)
        }

        if (t >= 1.0f) {
            transitioning = false;
        }

        float distToCylinder = sqrt(pow(endX - cylinderX, 1) + pow(endY - cylinderY, 1));
        if (distToCylinder < CYLINDER_RADIUS + 0.2f && clawAngle < 30.0f && !clawHolding) {
            clawHolding = true;
        }
        if (clawHolding && clawAngle < 25.0f) {
            cylinderX = (clawLeftMidX + clawRightMidX) / 2.0f;
            cylinderY = (clawLeftMidY + clawRightMidY) / 2.0f;
            cylinderZ = clawBaseZ;
            cylinderVelX = cylinderVelY = cylinderVelZ = 0.0f; // Reset velocity when held
        }
        if (clawAngle > 30.0f && clawHolding) {
            clawHolding = false;
        }
        if (!clawHolding) {
            cylinderVelY -= GRAVITY * DT;
            cylinderX += cylinderVelX * DT;
            cylinderY += cylinderVelY * DT;
            if (cylinderY <= 0.0f) {
                cylinderY = 0.0f;
                cylinderVelY = 0.0f;
            }
            // Apply friction to horizontal velocity
            cylinderVelX *= 0.95f;
            if (abs(cylinderVelX) < 0.01f) cylinderVelX = 0.0f;
        }
    }

    float torque1, torque2, torque3;
    computeJointTorques(torque1, torque2, torque3);

    float alpha1 = torque1 / I1;
    float alpha2 = torque2 / I2;
    float alpha3 = torque3 / I3;

    omega1 += alpha1 * DT;
    omega2 += alpha2 * DT;
    omega3 += alpha3 * DT;

    currentTheta1 += omega1 * DT;
    currentTheta2 += omega2 * DT;
    currentTheta3 += omega3 * DT;

    currentTheta1 = std::max(JOINT_LIMIT_MIN, std::min(JOINT_LIMIT_MAX, currentTheta1));
    currentTheta2 = std::max(JOINT_LIMIT_MIN, std::min(JOINT_LIMIT_MAX, currentTheta2));
    currentTheta3 = std::max(JOINT_LIMIT_MIN, std::min(JOINT_LIMIT_MAX, currentTheta3));

    if (currentTheta1 <= JOINT_LIMIT_MIN || currentTheta1 >= JOINT_LIMIT_MAX) omega1 = 0.0f;
    if (currentTheta2 <= JOINT_LIMIT_MIN || currentTheta2 >= JOINT_LIMIT_MAX) omega2 = 0.0f;
    if (currentTheta3 <= JOINT_LIMIT_MIN || currentTheta3 >= JOINT_LIMIT_MAX) omega3 = 0.0f;

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
    glutCreateWindow("Robotic Arm with Claw and Cylinder");
    init();
    glutDisplayFunc(display);
    glutMotionFunc(mouseMotion);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(16, update, 0);
    glutMainLoop();
    return 0;
}