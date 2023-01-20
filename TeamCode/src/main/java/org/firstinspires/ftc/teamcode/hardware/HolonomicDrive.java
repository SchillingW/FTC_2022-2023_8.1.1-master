package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RotateConvert;
import org.firstinspires.ftc.teamcode.util.VectorRotate;

public class HolonomicDrive {

    public Telemetry tele;

    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;

    public RotateConvert convertFL;
    public RotateConvert convertFR;
    public RotateConvert convertBL;
    public RotateConvert convertBR;

    public HolonomicDrive(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR,
                          RotateConvert convertFL, RotateConvert convertFR,
                          RotateConvert convertBL, RotateConvert convertBR,
                          Telemetry tele) {

        this.tele = tele;

        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorBL = motorBL;
        this.motorBR = motorBR;

        this.convertFL = convertFL;
        this.convertFR = convertFR;
        this.convertBL = convertBL;
        this.convertBR = convertBR;
    }

    public void run(double x, double y, double rot) {

        double speedFL = +x +y -rot;
        double speedFR = -x +y +rot;
        double speedBL = -x +y -rot;
        double speedBR = +x +y +rot;

        motorFL.setPower(speedFL * convertFL.polar);
        motorFR.setPower(speedFR * convertFR.polar);
        motorBL.setPower(speedBL * convertBL.polar);
        motorBR.setPower(speedBR * convertBR.polar);

        tele.addData("input x", x);
        tele.addData("input y", y);
        tele.addData("input rot", rot);
        tele.addData("motor fl", speedFL);
        tele.addData("motor fr", speedFR);
        tele.addData("motor bl", speedBL);
        tele.addData("motor br", speedBR);
    }

    public void run(double x, double y, double rot, double currRot) {

        run(VectorRotate.rotX(x, y, -currRot), VectorRotate.rotY(x, y, -currRot), rot);
    }

    public void stop() {

        run(0, 0, 0);
    }
}
