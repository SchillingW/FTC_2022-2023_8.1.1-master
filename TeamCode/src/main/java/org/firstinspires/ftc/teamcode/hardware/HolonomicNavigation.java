package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.InterpolateClamp;

public class HolonomicNavigation {

    public Telemetry tele;

    public HolonomicDrive drive;
    public HolonomicOdometry odometry;

    public InterpolateClamp approach;

    public double errorMarginLin;
    public double errorMarginRot;

    public double linearSpeedFactor;
    public double turnSpeedFactor;
    public double turnToLinearFactor;

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
    }

    public void goTo(double x, double y, double rot) {

        double diffX = x - odometry.currX;
        double diffY = y - odometry.currY;
        double diffRot = (rot - odometry.currRot) * turnToLinearFactor;

        double currentMagnitude = Math.sqrt(diffX * diffX + diffY * diffY + diffRot * diffRot);
        double targetMagnitude = approach.perform(currentMagnitude);

        diffX *= targetMagnitude / currentMagnitude;
        diffY *= targetMagnitude / currentMagnitude;
        diffRot *= targetMagnitude / currentMagnitude;

        drive.run(
                diffX * linearSpeedFactor, diffY * linearSpeedFactor,
                diffRot * turnSpeedFactor, odometry.currRot);
    }

    public boolean isDone(double x, double y, double rot) {

        return
                Math.abs(x - odometry.currX) <= errorMarginLin &&
                Math.abs(y - odometry.currY) <= errorMarginLin &&
                Math.abs(rot - odometry.currRot) <= errorMarginRot;
    }
}
