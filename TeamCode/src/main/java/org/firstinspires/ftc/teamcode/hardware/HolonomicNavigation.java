package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.InterpolateClamp;
import org.firstinspires.ftc.teamcode.util.VectorRotate;

public class HolonomicNavigation extends AutonomousSystem {

    public Telemetry tele;

    public HolonomicDrive drive;
    public HolonomicOdometry odometry;

    public InterpolateClamp approach;

    public double errorMarginLin;
    public double errorMarginRot;

    public double linearSpeedFactor;
    public double turnSpeedFactor;
    public double turnToLinearFactor;

    public double targetX;
    public double targetY;
    public double targetRot;

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

        setTarget(odometry.currX, odometry.currY, odometry.currRot);
    }

    @Override
    public void update() {

        super.update();

        track();

        if (isDone()) {

            stop();

        } else {

            double diffX = targetX - odometry.currX;
            double diffY = targetY - odometry.currY;
            double diffRot = (targetRot - odometry.currRot) * turnToLinearFactor;

            double currentMagnitude = Math.sqrt(diffX * diffX + diffY * diffY + diffRot * diffRot);
            double targetMagnitude = approach.perform(currentMagnitude);

            diffX *= targetMagnitude / currentMagnitude;
            diffY *= targetMagnitude / currentMagnitude;
            diffRot *= targetMagnitude / currentMagnitude;

            drive.run(
                    diffX * linearSpeedFactor, diffY * linearSpeedFactor,
                    diffRot * turnSpeedFactor, odometry.currRot);
        }
    }

    @Override
    public boolean isDone() {

        boolean done = super.isDone();

        return
                done &&
                Math.abs(targetX - odometry.currX) <= errorMarginLin &&
                Math.abs(targetY - odometry.currY) <= errorMarginLin &&
                Math.abs(targetRot - odometry.currRot) <= errorMarginRot;
    }

    public void setTarget(double x, double y, double rot) {

        targetX = x;
        targetY = y;
        targetRot = rot;
    }

    public void setTarget(double localX, double localY, double globalX, double globalY, double rot) {

        setTarget(
                VectorRotate.anchoredX(localX, localY, globalX, rot),
                VectorRotate.anchoredY(localX, localY, globalY, rot),
                rot);
    }

    public void track() {

        odometry.track();
    }

    public void stop() {

        drive.stop();
    }
}
