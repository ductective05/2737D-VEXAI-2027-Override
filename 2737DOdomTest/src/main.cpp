/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       2737d                                                     */
/*    Created:      1/17/2026                                                 */
/*    Description:  Odometry template - 2 vertical + 1 horizontal tracking    */
/*                  wheel, heading derived from the vertical wheel pair with  */
/*                  optional inertial cross-check.                            */
/*                                                                            */
/*  CONVENTIONS (read this before tuning anything):                           */
/*    - Field frame is standard math: heading 0 rad = +X axis, CCW positive.  */
/*      Robot forward unit vector = (cos h, sin h). Robot right = (sin h,-cos h)|
/*    - Local robot frame: +y = forward, +x = right (strafe).                 */
/*    - LVert_offset  = distance from tracking center to the LEFT  wheel      */
/*    - RVert_offset  = distance from tracking center to the RIGHT wheel      */
/*    - Horiz_offset  = distance from tracking center to the horizontal wheel,|
/*      positive if that wheel sits FORWARD of the tracking center.           */
/*    - All distances in inches.                                              */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <stdio.h>
#include <string>
#include <cstring>
#include <vector>
#include <sstream>
#include <cstdlib>  // atof

using namespace std;
using namespace vex;
using signature = vision::signature;
using code = vision::code;

const double PI = 3.14159265358979323846;

// ===========================================================================
//                               HARDWARE
// ===========================================================================
vex::brain Brain;
controller Controller1 = controller(primary);

// inertial
inertial  Inertial0           = inertial(PORT7);

// Tracking wheels. Flip the bool if a wheel counts backwards; verify with the
// live readout in odomTask before you trust anything downstream.
rotation  Rotation_vertL      = rotation(PORT18, true);   // left  vertical
rotation  Rotation_vertR      = rotation(PORT19, false);  // right vertical
rotation  Rotation_horizontal = rotation(PORT17, false);  // strafe

// --- drivetrain ------------------------------------------------------------
motor leftMotor1  = motor(PORT2,  ratio6_1, false);
motor leftMotor2  = motor(PORT4,  ratio6_1, true);
motor leftMotor3  = motor(PORT6,  ratio6_1, false);
motor rightMotor1 = motor(PORT1,  ratio6_1, true);
motor rightMotor2 = motor(PORT3,  ratio6_1, false);
motor rightMotor3 = motor(PORT13, ratio6_1, true);
motor intakeMotor1 = motor(PORT11, ratio6_1, true);
motor intakeMotor2 = motor(PORT12, ratio6_1, false);

motor_group LeftDrive  = motor_group(leftMotor1,  leftMotor2,  leftMotor3);
motor_group RightDrive = motor_group(rightMotor1, rightMotor2, rightMotor3);
motor_group Intake     = motor_group(intakeMotor1, intakeMotor2);

smartdrive driveTrain = smartdrive(LeftDrive, RightDrive, Inertial0,319.19, 320, 40, mm, 1.6666666666666667);

vex::mutex position_mutex;

// ===========================================================================
//                        TRACKING GEOMETRY (TUNE THESE)
// ===========================================================================
// Measure center-to-center of the two vertical wheels, then refine with
// trackWidthCalibration() below. A 1% error here is 3.6 deg of heading error
// per full robot rotation, so this is the single most important constant.
double trackingWheelDiameter = 2.00;   // vertical wheels, effective diameter
double horizWheelDiameter    = 2.00;   // horizontal wheel, effective diameter

double LVert_offset = 5.00;   // tracking center -> left  vertical wheel
double RVert_offset = 5.00;   // tracking center -> right vertical wheel
double Horiz_offset = 0.00;   // tracking center -> horizontal wheel (+ = fwd)

double inertial_ratio = 360.0 / 360.0;  // inertial scale correction

// ===========================================================================
//                            ODOMETRY VARIABLES
// ===========================================================================
double x_current     = 0.0;
double y_current     = 0.0;
double global_heading = PI / 2.0;   // radians, math convention

double prev_vertL    = 0.0;   // last raw reading, turns
double prev_vertR    = 0.0;
double prev_horiz    = 0.0;
double prev_heading  = PI / 2.0;

// inertial cross-check state
double prev_raw_rotation = 0.0;   // deg
double prev_bias_rotation = 0.0;  // deg, used only by the ZUPT estimator
double gyro_bias_rate     = 0.0;  // deg/s of drift while stationary
double prev_bias_time     = 0.0;  // s
double prev_odom_time     = 0.0;  // s

// diagnostics: how far the wheel heading and the inertial have diverged (deg)
double heading_disagreement_deg = 0.0;

// ===========================================================================
//                          HEADING SOURCE CONFIG
// ===========================================================================
enum HeadingSource { HEADING_WHEELS, HEADING_INERTIAL, HEADING_FUSED };

// Start on HEADING_WHEELS while you calibrate the track width. Once the wheel
// heading closes a 10-rotation spin cleanly, switch to HEADING_FUSED.
HeadingSource heading_source = HEADING_WHEELS;

// Fusion weight for HEADING_FUSED: 0.0 = pure wheels, 1.0 = pure inertial.
// This is a per-tick blend of the two yaw-rate estimates, not a drift
// correction - it reduces noise, it does not fix a bad track width.
double heading_fusion_gain = 0.10;

// If the two sources disagree by more than this in a single tick, a tracking
// wheel almost certainly skipped or lifted, so trust the inertial for that
// tick. 3 deg/tick at 5 ms is 600 deg/s of disagreement - very generous.
double heading_slip_tolerance = 3.0 * PI / 180.0;


double clamp(double value, double minVal, double maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

// wrap to (-PI, PI]
double wrapRad(double a) {
    while (a >  PI) a -= 2.0 * PI;
    while (a <= -PI) a += 2.0 * PI;
    return a;
}

// Convert a compass-style heading (0 = field +Y / away from driver, CW
// positive) into this template's math heading in radians.
double mathFromCompass(double compassDeg) {
    return wrapRad((90.0 - compassDeg) * PI / 180.0);
}

// returns the current heading in radians, math convention (0 = +X, CCW positive)
double currentHeadingRad() {
    position_mutex.lock();
    double h = global_heading;
    position_mutex.unlock();
    return h;
}
// converts current heading to degrees
double currentHeadingDeg() { 
    return currentHeadingRad() * 180.0 / PI;
}

// returns true if robot is not moving,
bool robotIsStationary() {
    return fabs(Rotation_vertL.velocity(dps)) < 2.0 &&
           fabs(Rotation_vertR.velocity(dps)) < 2.0 &&
           fabs(Rotation_horizontal.velocity(dps)) < 2.0;
}

// ===========================================================================
//                         GYRO BIAS 
// ===========================================================================
// The inertial sensor can drift over time, when the robot is stationary heading should be constant so we use this function to counter the inertial drift
void updateGyroBias() {
    double t  = Brain.timer(msec) / 1000.0;
    double dt = t - prev_bias_time;
    if (dt < 0.05) return;             

    double raw  = Inertial0.rotation(degrees);
    double rate = (raw - prev_bias_rotation) / dt;   // deg/s

    prev_bias_time     = t;
    prev_bias_rotation = raw;

    if (robotIsStationary() && fabs(rate) < 5.0) {
        gyro_bias_rate = 0.98 * gyro_bias_rate + 0.02 * rate;
    }
}

// ===========================================================================
//                              ODOMETRY
// ===========================================================================
void updateOdometry() {
    // ---- 1. raw deltas, converted to inches of arc at each wheel ----------
    double vertL = Rotation_vertL.position(turns);
    double vertR = Rotation_vertR.position(turns);
    double horiz = Rotation_horizontal.position(turns);

    double dL = (vertL - prev_vertL) * PI * trackingWheelDiameter; //distance left
    double dR = (vertR - prev_vertR) * PI * trackingWheelDiameter; //distance right
    double dS = (horiz - prev_horiz) * PI * horizWheelDiameter; //distance horizontal

    prev_vertL = vertL;
    prev_vertR = vertR;
    prev_horiz = horiz;

    // ---- 2. heading change -----------------------------------------------
    // Right wheel arc  = dC + RVert_offset * dTheta
    // Left  wheel arc  = dC - LVert_offset * dTheta
    // so dTheta = (dR - dL) / (LVert_offset + RVert_offset), CCW positive.
    double track = LVert_offset + RVert_offset;
    double dTheta_wheel;
    if (track > 1e-6) {
        dTheta_wheel = (dR - dL) / track;
    } else {
        dTheta_wheel = 0.0;
    }
    // Inertial cross-check. Inertial0.rotation() is unbounded and clockwise
    // positive, so negate it to get CCW-positive math radians. Scaling is
    // applied to rotation(), never heading(), so it stays correct past 360.
    double t_now  = Brain.timer(msec) / 1000.0;
    double dt     = t_now - prev_odom_time;
    prev_odom_time = t_now;

    double raw_rot     = Inertial0.rotation(degrees);
    double raw_delta   = (raw_rot - prev_raw_rotation) - gyro_bias_rate * dt;
    prev_raw_rotation  = raw_rot;
    double dTheta_imu  = -raw_delta * inertial_ratio * PI / 180.0;

    heading_disagreement_deg += (dTheta_wheel - dTheta_imu) * 180.0 / PI;

    double dTheta;
    if (heading_source == HEADING_INERTIAL) {
        dTheta = dTheta_imu;
    } else if (heading_source == HEADING_WHEELS) {
        dTheta = dTheta_wheel;
    } else {
        if (fabs(dTheta_wheel - dTheta_imu) > heading_slip_tolerance) {
            dTheta = dTheta_imu;                 // a wheel slipped or lifted
        } else {
            dTheta = (1.0 - heading_fusion_gain) * dTheta_wheel
                   +        heading_fusion_gain  * dTheta_imu;
        }
    }

    // ---- 3. arc displacement in the local frame ---------------------------
    // Forward arc of the tracking center, weighted so both wheels contribute:
    //   dC = (LVert_offset*dR + RVert_offset*dL) / track
    // Lateral arc of the tracking center: the horizontal wheel also sees
    // -Horiz_offset*dTheta of pure rotation, so add it back.
    double dC  = (track > 1e-6)
               ? (LVert_offset * dR + RVert_offset * dL) / track
               : 0.5 * (dL + dR);
    double dSc = dS + Horiz_offset * dTheta;

    // Chord factor: over an arc, the straight-line displacement is
    // 2*R*sin(dTheta/2), and R = arc/dTheta, so scale by 2*sin(dTheta/2)/dTheta.
    // The limit as dTheta -> 0 is 1, which is the straight-line case.
    double local_y, local_x;   // forward, right
    if (fabs(dTheta) < 1e-9) {
        local_y = dC;
        local_x = dSc;
    } else {
        double k = 2.0 * sin(dTheta / 2.0) / dTheta;
        local_y = k * dC;
        local_x = k * dSc;
    }

    // ---- 4. rotate into the field frame at the mid-arc heading ------------
    position_mutex.lock();

    double heading_mid = global_heading + dTheta / 2.0;

    global_heading = wrapRad(global_heading + dTheta);
    prev_heading   = global_heading;

    // forward = (cos h, sin h); right = (sin h, -cos h)
    x_current += local_y * cos(heading_mid) + local_x * sin(heading_mid);
    y_current += local_y * sin(heading_mid) - local_x * cos(heading_mid);

    // field is 144 in; clamp to keep a bad frame from poisoning downstream math
    x_current = clamp(x_current, -6.0, 150.0);
    y_current = clamp(y_current, -6.0, 150.0);

    position_mutex.unlock();
}

// Mid-match resync against a wall or a known field feature. Pass whichever
// values you actually know; heading is in DEGREES, math convention.
void resetOdom(double x, double y, double headingDeg) {
    position_mutex.lock();
    x_current      = x;
    y_current      = y;
    global_heading = wrapRad(headingDeg * PI / 180.0);
    prev_heading   = global_heading;
    position_mutex.unlock();
}

// Heading-only resync. Does NOT touch rotation sensor accumulators, so the
// next updateOdometry tick still sees a sane delta.
void resetHeading(double headingDeg) {
    position_mutex.lock();
    global_heading = wrapRad(headingDeg * PI / 180.0);
    prev_heading   = global_heading;
    position_mutex.unlock();
    heading_disagreement_deg = 0.0;
}

// ===========================================================================
//                              ODOM TASK
// ===========================================================================
int odomTask() {
    int frame = 0;
    while (true) {
        updateGyroBias();
        updateOdometry();

        // Screen writes are slow. Print at ~10 Hz, run the math at 200 Hz.
        if (++frame >= 20) {
            frame = 0;
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.clearLine();
            Brain.Screen.print("X: %.2f", x_current);

            Brain.Screen.setCursor(2, 1);
            Brain.Screen.clearLine();
            Brain.Screen.print("Y: %.2f", y_current);

            Brain.Screen.setCursor(3, 1);
            Brain.Screen.clearLine();
            Brain.Screen.print("H: %.2f deg", currentHeadingDeg());

            // If this number walks away from 0, your track width is wrong or
            // a tracking wheel is slipping.
            Brain.Screen.setCursor(4, 1);
            Brain.Screen.clearLine();
            Brain.Screen.print("wheel-imu: %.2f deg", heading_disagreement_deg);
        }

        vex::wait(5, msec);
    }
    return 0;
}

// ===========================================================================
//                        TRACK WIDTH CALIBRATION
// ===========================================================================
// Push the robot through exactly N full rotations by hand, in place, then read
// the suggested effective track width off the screen. Call it from main()
// instead of your auton, run it once, hard-code the result into
// LVert_offset / RVert_offset (split it however your wheels actually sit).
void trackWidthCalibration(int rotations = 10) {
    Rotation_vertL.resetPosition();
    Rotation_vertR.resetPosition();

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Spin robot %d turns CCW, then read:", rotations);

    while (true) {
        double dL = Rotation_vertL.position(turns) * PI * trackingWheelDiameter;
        double dR = Rotation_vertR.position(turns) * PI * trackingWheelDiameter;
        double totalTheta = 2.0 * PI * rotations;

        Brain.Screen.setCursor(3, 1);
        Brain.Screen.clearLine();
        Brain.Screen.print("dL %.2f  dR %.2f", dL, dR);

        Brain.Screen.setCursor(4, 1);
        Brain.Screen.clearLine();
        Brain.Screen.print("track width = %.4f in", (dR - dL) / totalTheta);

        vex::wait(100, msec);
    }
}

// ===========================================================================
//                                PREMATCH
// ===========================================================================
// startHeadingDeg is in MATH degrees (0 = +X, CCW positive). If you think in
// compass headings, call preMatch(mathFromCompass(180), x, y).
void preMatch(double startHeadingDeg = 90.0,
              double startX = 0.0,
              double startY = 0.0) {

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("preMatch: configuring motors...");

    // ---- motors -----------------------------------------------------------
    LeftDrive.setStopping(brake);
    RightDrive.setStopping(brake);
    LeftDrive.setVelocity(100, percent);
    RightDrive.setVelocity(100, percent);
    LeftDrive.setMaxTorque(100, percent);
    RightDrive.setMaxTorque(100, percent);
    LeftDrive.resetPosition();
    RightDrive.resetPosition();
    LeftDrive.stop();
    RightDrive.stop();

    Intake.setStopping(coast);
    Intake.setVelocity(100, percent);
    Intake.setMaxTorque(100, percent);
    Intake.resetPosition();
    Intake.stop();

    driveTrain.setDriveVelocity(80, percent);
    driveTrain.setTurnVelocity(50, percent);
    driveTrain.setStopping(brake);

    // ---- tracking wheels --------------------------------------------------
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("checking tracking wheels...");

    if (!Rotation_vertL.installed())      Controller1.Screen.print("NO VERT-L ");
    if (!Rotation_vertR.installed())      Controller1.Screen.print("NO VERT-R ");
    if (!Rotation_horizontal.installed()) Controller1.Screen.print("NO HORIZ ");

    Rotation_vertL.resetPosition();
    Rotation_vertR.resetPosition();
    Rotation_horizontal.resetPosition();

    // ---- inertial ---------------------------------------------------------
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("calibrating inertial - DO NOT MOVE");

    Inertial0.calibrate();
    double calStart = Brain.timer(msec);
    while (Inertial0.isCalibrating() && (Brain.timer(msec) - calStart) < 5000.0) {
        vex::wait(20, msec);
    }
    vex::wait(200, msec);   // let the filter settle after calibrate() returns
    Inertial0.setRotation(0, degrees);
    Inertial0.setHeading(0, degrees);

    // ---- stationary gyro bias sample (1 s) --------------------------------
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("sampling gyro bias...");

    double biasStartRot  = Inertial0.rotation(degrees);
    double biasStartTime = Brain.timer(msec);
    vex::wait(1000, msec);
    double biasElapsed   = (Brain.timer(msec) - biasStartTime) / 1000.0;
    gyro_bias_rate = (Inertial0.rotation(degrees) - biasStartRot) / biasElapsed;
    if (fabs(gyro_bias_rate) > 2.0) gyro_bias_rate = 0.0;  // reject a bumped bot

    // ---- odometry state ---------------------------------------------------
    // Snapshot the accumulators AFTER every reset so the first tick sees a
    // delta of zero rather than the whole pre-match history.
    prev_vertL = Rotation_vertL.position(turns);
    prev_vertR = Rotation_vertR.position(turns);
    prev_horiz = Rotation_horizontal.position(turns);

    prev_raw_rotation  = Inertial0.rotation(degrees);
    prev_bias_rotation = prev_raw_rotation;
    prev_bias_time     = Brain.timer(msec) / 1000.0;
    prev_odom_time     = prev_bias_time;

    heading_disagreement_deg = 0.0;

    position_mutex.lock();
    x_current      = startX;
    y_current      = startY;
    global_heading = wrapRad(startHeadingDeg * PI / 180.0);
    prev_heading   = global_heading;
    position_mutex.unlock();

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("READY  x %.1f y %.1f h %.1f  bias %.3f d/s",
                       startX, startY, startHeadingDeg, gyro_bias_rate);
    Controller1.rumble(".");
}

// ===========================================================================
//                                  MAIN
// ===========================================================================
int main() {
    preMatch(180);          

    task odom(odomTask);

    // trackWidthCalibration(10);   // uncomment to calibrate, comment out auton

    while (true) {
        vex::wait(20, msec);
    }
}