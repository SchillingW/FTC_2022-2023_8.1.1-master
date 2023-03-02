package org.firstinspires.ftc.teamcode.OLDPROJECT.hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class MecDriveFlip extends MecanumDrive {

    float coefFL;
    float coefFR;
    float coefBL;
    float coefBR;

    public MecDriveFlip(
            Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight,
            float coefFL, float coefFR, float coefBL, float coefBR) {

        super(frontLeft, frontRight, backLeft, backRight);

        this.coefFL = coefFL;
        this.coefFR = coefFR;
        this.coefBL = coefBL;
        this.coefBR = coefBR;
    }

    @Override
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle) {
        super.driveFieldCentric(forwardSpeed, strafeSpeed, turnSpeed, -gyroAngle);
    }

    @Override
    public void driveWithMotorPowers(
            double frontLeftSpeed, double frontRightSpeed,
            double backLeftSpeed, double backRightSpeed) {

        super.driveWithMotorPowers(
                frontLeftSpeed * coefFL, frontRightSpeed * coefFR,
                backLeftSpeed * coefBL, backRightSpeed * coefBR);
    }
}
