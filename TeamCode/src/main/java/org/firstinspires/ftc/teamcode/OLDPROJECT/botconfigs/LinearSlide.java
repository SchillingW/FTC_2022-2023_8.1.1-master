package org.firstinspires.ftc.teamcode.OLDPROJECT.botconfigs;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide
{
    private Motor slide;
    private Servo claw;

    public double minMagnitude = 0.2;
    public double maxMagnitude = 1;
    private double slideZeroMag = -0.1;

    private int errorMargin = 60;
    private double armSpeed = 0.75;

    public int ground = 0; public int low = -1275;
    public int med = -2150; public int high = -2925;
    public int driveHeight = -250;
    public int stackDriveHeight = -600;
    public int[] stacks = {-405, -320, -160, -65};
    public int[] slidePositions = {ground, low, med, high};

    public double extraTime = 0.25;

    public LinearSlide(Telemetry tele, HardwareMap map)
    {
        claw = map.servo.get("claw");

        slide = new Motor(map, "slide");
        slide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void goTo(int reachHeight, Telemetry telemetry)
    {
        if(isAtTarget(reachHeight))
        {
            slide.set(slideZeroMag);
            return;
        }

        double slideVector = reachHeight - slide.encoder.getPosition();
        double currentMagnitude = Math.abs(slideVector);
        double targetMagnitude = maxMagnitude;

        slideVector *= targetMagnitude/currentMagnitude;
        slide.set(slideVector);

        telemetry.addData("Current Slide Pos", slide.encoder.getPosition());
        telemetry.addData("Desired Slide Pos", reachHeight);
        telemetry.addData("Current Slide Magnitude", currentMagnitude);
        telemetry.addData("Target Slide Magnitude", targetMagnitude);
        telemetry.update();
    }

    public boolean isOpen(){return claw.getPosition() > 0.1;}

    public void goToAuto(int reachHeight, double speed, Telemetry telemetry)
    {
        if(isAtTarget(reachHeight))
        {
            slide.set(slideZeroMag);
            return;
        }

        double slideVector = reachHeight - slide.encoder.getPosition();
        double currentMagnitude = Math.abs(slideVector);
        double targetMagnitude = maxMagnitude * speed;

        slideVector *= targetMagnitude/currentMagnitude;
        slide.set(slideVector);

        telemetry.addData("Current Slide Pos", slide.encoder.getPosition());
        telemetry.addData("Desired Slide Pos", reachHeight);
        telemetry.addData("Current Slide Magnitude", currentMagnitude);
        telemetry.addData("Target Slide Magnitude", targetMagnitude);
        telemetry.update();
    }

    public void goToFull(int reachHeight, Telemetry tele, LinearOpMode mode) {

        if (mode.opModeIsActive()) {

            while (!isAtTarget(reachHeight) && mode.opModeIsActive()) {

                goTo(reachHeight, tele);
            }

            ElapsedTime time = new ElapsedTime();

            while (time.seconds() < extraTime && mode.opModeIsActive()) {

                goTo(reachHeight, tele);
            }

            slide.set(slideZeroMag);
        }
    }

    public void moveByJoystick(double mag) {slide.set((mag == 0) ? slideZeroMag : mag * armSpeed);}
    public void setSlide(double mag){slide.set(mag);}
    public boolean isAtTarget(int reachHeight) {return (Math.abs(slide.encoder.getPosition() - reachHeight) < errorMargin);}
    public int getCurrentPos(){return slide.encoder.getPosition();}
    public void openClaw() {claw.setPosition(0.45);}
    public void middleClaw() {claw.setPosition(0.225);}
    public void closeClaw() {claw.setPosition(0);}
}
