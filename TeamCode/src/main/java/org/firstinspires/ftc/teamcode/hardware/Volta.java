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

        RotateConvert convertMotor = new RotateConvert(420, 9);
        RotateConvert convertEncoder = new RotateConvert(8192, 5);

        HolonomicDrive drive = new HolonomicDrive(
                motorFL, motorFR, motorBL, motorBR,
                convertMotor.instance(1), convertMotor.instance(1),
                convertMotor.instance(1), convertMotor.instance(1),
                tele);

        HolonomicOdometry odometry = new HolonomicOdometry(
                motorFL, motorFR, motorBL,
                convertEncoder.instance(1), convertEncoder.instance(1),
                convertEncoder.instance(1),
                -4, 4, -2,
                tele);

         nav = new HolonomicNavigation(drive, odometry, tele);
    }
}
