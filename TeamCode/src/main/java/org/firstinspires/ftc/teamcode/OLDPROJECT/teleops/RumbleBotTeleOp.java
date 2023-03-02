package org.firstinspires.ftc.teamcode.OLDPROJECT.teleops;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OLDPROJECT.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.OLDPROJECT.botconfigs.PursuitBotTesting;
import org.firstinspires.ftc.teamcode.OLDPROJECT.hardware.GamepadSystem;

@TeleOp(name="RumbleBotTeleOp", group="ClawLiftBot")
public class RumbleBotTeleOp extends OpMode {

    PursuitBotTesting robot;
    LinearSlide linearSlide;
    ColorRangeSensor sensor;
    DistanceSensor sensorDistance;

    public double turnSpeed = 0.6;
    public double linearSpeed = 0.75;

    public int heightIndex = 0;
    public boolean joystickControl = false;

    public boolean lastUp;
    public boolean lastDown;

    public double baseHeading;

    public boolean cyclingMode;
    public boolean autoRaiseClaw;
    public boolean clawClosed;
    public ElapsedTime timer;


    // input system reference
    GamepadSystem input;
    final double SCALE_FACTOR = 255;
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // called on program initialization
    @Override
    public void init() {

        robot = new PursuitBotTesting(telemetry, hardwareMap);
        input = new GamepadSystem(this);
        linearSlide = new LinearSlide(telemetry, hardwareMap);
        sensor = (ColorRangeSensor) hardwareMap.colorSensor.get("sensor");
        linearSlide.goTo(linearSlide.stacks[3], telemetry);
        linearSlide.openClaw();
        heightIndex = 0;
        autoRaiseClaw = false;
        cyclingMode = false;
        clawClosed = false;
        timer = new ElapsedTime();
        timer.reset();

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor");

    }

    // called repeatedly during program
    @Override
    public void loop() {
        telemetry.addData("Drive Speed", linearSpeed);

        // NORMAL
        robot.drive.driveRobotCentric(
                input.gamepad1.getLeftY() * Math.abs(input.gamepad1.getLeftY()) * linearSpeed,
                input.gamepad1.getLeftX() * Math.abs(input.gamepad1.getLeftX()) * linearSpeed,
                input.gamepad1.getRightX() * Math.abs(input.gamepad1.getRightX()) * turnSpeed);


        // GYRO COMP

        /*robot.odometry.update();
        robot.drive.driveFieldCentric(
                input.gamepad1.getLeftY() * Math.abs(input.gamepad1.getLeftY()) * linearSpeed,
                input.gamepad1.getLeftX() * Math.abs(input.gamepad1.getLeftX()) * linearSpeed,
                input.gamepad1.getRightX() * Math.abs(input.gamepad1.getRightX()) * turnSpeed,
                robot.odometry.getPose().getHeading() / 2 / Math.PI * 360 - baseHeading);*/



        // reorient forward to current direction
        if (gamepad1.right_bumper) baseHeading = robot.odometry.getPose().getHeading() / 2 / Math.PI * 360;

        telemetry.addData("Cycling Mode", cyclingMode);
        telemetry.addData("heading", robot.odometry.getPose().getHeading() / 2 / Math.PI * 360);
        telemetry.addData("base heading", baseHeading);
        /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());*/


        if(input.gamepad1.getButton(GamepadKeys.Button.B))
        {
            linearSpeed = 1;
        }

        if(input.gamepad1.getButton(GamepadKeys.Button.Y))
        {
            linearSpeed = 0.825;
        }
//
        if(input.gamepad1.getButton(GamepadKeys.Button.X))
        {
            linearSpeed = 0.75;
        }
//
        if (input.gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            linearSlide.closeClaw();
            telemetry.addData("servo", "close");
            telemetry.update();
        }
        //full open=0
        if (input.gamepad2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            linearSlide.openClaw();
            telemetry.addData("servo", "open");
            telemetry.update();
        }

        if (sensorDistance.getDistance(DistanceUnit.CM) <= 8) {
            gamepad2.rumble(500);
        }

        boolean highUp = input.gamepad2.getButton(GamepadKeys.Button.X);
        boolean thisUp = input.gamepad2.getButton(GamepadKeys.Button.DPAD_UP);
        boolean thisDown = input.gamepad2.getButton(GamepadKeys.Button.DPAD_DOWN);
        if (thisUp && !lastUp) {heightIndex++; joystickControl = false;}
        if (thisDown & !lastDown) {heightIndex--; joystickControl = false;}
        if (highUp) { heightIndex = 3;}
        heightIndex = Math.max(0, Math.min(linearSlide.slidePositions.length - 1, heightIndex));
        telemetry.addData("heightIndex", heightIndex);
        lastUp = thisUp;
        //lastDown = thisDown;

        if (input.gamepad2.getRightY() != 0 || joystickControl)
        {
            linearSlide.moveByJoystick(input.gamepad2.getRightY() * 1.5);
            joystickControl = true;
        }
        else
        {
            if(heightIndex == 3) linearSlide.goTo(linearSlide.high + 30, telemetry);
            else linearSlide.goTo(linearSlide.slidePositions[heightIndex], telemetry);
        }


        if(cyclingMode)
        {
            linearSpeed = 0.4;
            if(!clawClosed) {
                joystickControl = true;
                linearSlide.moveByJoystick(1);
            }

            else joystickControl = false;
        }


        if(autoRaiseClaw && cyclingMode && timer.seconds() >= 0.5)
        {
            autoRaiseClaw = false;
            heightIndex = 3;
        }
    }


}
