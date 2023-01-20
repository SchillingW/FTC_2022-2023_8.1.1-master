package org.firstinspires.ftc.teamcode.bot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.DriveSlide;
import org.firstinspires.ftc.teamcode.hardware.HolonomicDrive;
import org.firstinspires.ftc.teamcode.hardware.HolonomicNavigation;
import org.firstinspires.ftc.teamcode.hardware.HolonomicOdometry;
import org.firstinspires.ftc.teamcode.hardware.LinearSlide;
import org.firstinspires.ftc.teamcode.util.InterpolateClamp;
import org.firstinspires.ftc.teamcode.util.RotateConvert;

public class Volta extends DriveSlide {

    public Claw claw;

    public Volta(HardwareMap map, Telemetry tele) {

        super(null, null, tele);



        DcMotor motorFL = map.get(DcMotor.class, "motorFL");
        DcMotor motorFR = map.get(DcMotor.class, "motorFR");
        DcMotor motorBL = map.get(DcMotor.class, "motorBL");
        DcMotor motorBR = map.get(DcMotor.class, "motorBR");

        // encoderL & encoderR record 15.8rot over a 118in linear path
        RotateConvert convertMotor = new RotateConvert(420, 1);
        RotateConvert convertEncoder = new RotateConvert(8192, 7.468);

        HolonomicDrive drive = new HolonomicDrive(
                motorFL, motorFR, motorBL, motorBR,
                convertMotor.instance(1), convertMotor.instance(-1),
                convertMotor.instance(1), convertMotor.instance(-1),
                tele);

        // encoderL records +114in over a 10PIrad rotation
        // encoderR records -108in over a 10PIrad rotation
        // encoderH records -047in over a 10PIrad rotation
        HolonomicOdometry odometry = new HolonomicOdometry(
                motorFL, motorFR, motorBL,
                convertEncoder.instance(1), convertEncoder.instance(-1),
                convertEncoder.instance(-1),
                -3.628,
                3.438,
                -1.496,
                tele);

        InterpolateClamp approach = new InterpolateClamp(
                6, 24,
                0.25, 0.5);

        nav = new HolonomicNavigation(
                drive, odometry, approach,
                0.5, Math.PI / 96,
                1, 1, 48 / Math.PI,
                tele);
        subsystem.set(0, nav);



        DcMotor motorSlide = map.get(DcMotor.class, "motorSlide");

        // motorSlide records 2.69rot over a 24in displacement
        RotateConvert convertSlide = new RotateConvert(756, 8.922, -1);

        InterpolateClamp approachBelow = new InterpolateClamp(
                3, 6,
                0.5, 1);

        InterpolateClamp approachAbove = new InterpolateClamp(
                6, 12,
                0.25, 0.5);

        slide = new LinearSlide(
                motorSlide, convertSlide,
                approachBelow, approachAbove,
                0.25, 1, 0.125,
                tele);
        subsystem.set(1, slide);



        Servo servoClaw = map.get(Servo.class, "servoClaw");

        claw = new Claw(servoClaw, 0, 0.5, tele);
    }
}
