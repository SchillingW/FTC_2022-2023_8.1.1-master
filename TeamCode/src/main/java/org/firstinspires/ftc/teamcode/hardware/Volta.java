package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RotateConvert;

public class Volta {

    public Telemetry tele;

    public HolonomicNavigation nav;

    public Volta(HardwareMap map, Telemetry tele) {

        this.tele = tele;

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

        nav = new HolonomicNavigation(drive, odometry, tele);
    }
}
