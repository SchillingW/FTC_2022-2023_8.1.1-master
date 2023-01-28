package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.InterpolateClamp;
import org.firstinspires.ftc.teamcode.util.VectorRotate;

// navigation system with drive train and odometry
public class HolonomicNavigation extends AutonomousSystem {

    // store telemetry device
    public Telemetry tele;

    // declare subsystems
    public HolonomicDrive drive;
    public HolonomicOdometry odometry;

    // declare approach speed gradient
    public InterpolateClamp approach;

    // acceptable range from target
    public double errorMarginLin;
    public double errorMarginRot;

    // directional speed multipliers
    public double linearSpeedFactor;
    public double turnSpeedFactor;
    public double turnToLinearFactor;

    // declare current target position
    public double targetX;
    public double targetY;
    public double targetRot;

    // initialize device
    public HolonomicNavigation(HolonomicDrive drive, HolonomicOdometry odometry,
                               InterpolateClamp approach,
                               double errorMarginLin, double errorMarginRot,
                               double linearSpeedFactor, double turnSpeedFactor,
                               double turnToLinearFactor,
                               Telemetry tele) {

        this.tele = tele;

        this.drive = drive;
        this.odometry = odometry;

        this.approach = approach;

        this.errorMarginLin = errorMarginLin;
        this.errorMarginRot = errorMarginRot;

        this.linearSpeedFactor = linearSpeedFactor;
        this.turnSpeedFactor = turnSpeedFactor;
        this.turnToLinearFactor = turnToLinearFactor;

        // set target to current position
        setTarget(odometry.currX, odometry.currY, odometry.currRot);
    }

    // move towards target
    @Override
    public void update() {

        // update subsystems
        super.update();

        // track current position
        track();

        // hold position if reached
        if (isDone()) {

            stop();

        } else {

            // get displacement from target
            double diffX = targetX - odometry.currX;
            double diffY = targetY - odometry.currY;
            double diffRot = (targetRot - odometry.currRot) * turnToLinearFactor;

            // magnitude of vector towards target
            double currentMagnitude = Math.sqrt(diffX * diffX + diffY * diffY + diffRot * diffRot);

            // desired magnitude by approach gradient
            double targetMagnitude = approach.perform(currentMagnitude);

            // scale displacement to target magnitude
            diffX *= targetMagnitude / currentMagnitude;
            diffY *= targetMagnitude / currentMagnitude;
            diffRot *= targetMagnitude / currentMagnitude;

            // run device
            drive.run(
                    diffX * linearSpeedFactor, diffY * linearSpeedFactor,
                    diffRot * turnSpeedFactor, odometry.currRot);
        }
    }

    // return true if at target
    @Override
    public boolean isDone() {

        // true if all subsystems are done
        boolean done = super.isDone();

        // true if current position in range of target
        return
                done &&
                Math.abs(targetX - odometry.currX) <= errorMarginLin &&
                Math.abs(targetY - odometry.currY) <= errorMarginLin &&
                Math.abs(targetRot - odometry.currRot) <= errorMarginRot;
    }

    // set target position
    public void setTarget(double x, double y, double rot) {

        targetX = x;
        targetY = y;
        targetRot = rot;
    }

    // set target position such that local point on bot lies over global point
    public void setTarget(double localX, double localY,
                          double globalX, double globalY,
                          double rot) {

        setTarget(localX, localY, globalX, globalY, rot, rot);
    }

    // set target position such that local point on bot lies over global point
    public void setTarget(double localX, double localY,
                          double globalX, double globalY,
                          double imagineRot, double actualRot) {

        setTarget(
                VectorRotate.anchoredX(localX, localY, globalX, imagineRot),
                VectorRotate.anchoredY(localX, localY, globalY, imagineRot),
                actualRot);
    }

    // track device position
    public void track() {

        odometry.track();
    }

    // stop navigation system
    public void stop() {

        drive.stop();
    }
}
