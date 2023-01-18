package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RotateConvert;
import org.firstinspires.ftc.teamcode.util.VectorRotate;

public class HolonomicOdometry {

    public Telemetry tele;

    public DcMotor encoderL;
    public DcMotor encoderR;
    public DcMotor encoderH;

    public RotateConvert convertL;
    public RotateConvert convertR;
    public RotateConvert convertH;

    public double offsetL;
    public double offsetR;
    public double offsetH;

    public double currX;
    public double currY;
    public double currRot;

    public double lastL;
    public double lastR;
    public double lastH;

    public HolonomicOdometry(DcMotor encoderL, DcMotor encoderR, DcMotor encoderH,
                             RotateConvert convertL, RotateConvert convertR,
                             RotateConvert convertH,
                             double offsetL, double offsetR, double offsetH,
                             Telemetry tele) {

        this.tele = tele;

        this.encoderL = encoderL;
        this.encoderR = encoderR;
        this.encoderH = encoderH;

        this.convertL = convertL;
        this.convertR = convertR;
        this.convertH = convertH;

        this.offsetL = offsetL;
        this.offsetR = offsetR;
        this.offsetH = offsetH;

        this.lastL = encoderL.getCurrentPosition() / convertL.tickPerInch;
        this.lastR = encoderR.getCurrentPosition() / convertR.tickPerInch;
        this.lastH = encoderH.getCurrentPosition() / convertH.tickPerInch;
    }

    public void update() {

        double newL = encoderL.getCurrentPosition() / convertL.tickPerInch;
        double newR = encoderR.getCurrentPosition() / convertR.tickPerInch;
        double newH = encoderH.getCurrentPosition() / convertH.tickPerInch;

        double diffL = newL - lastL;
        double diffR = newR - lastR;
        double diffH = newH - lastH;

        double[] displace = calcDisplace(
                diffL, diffR, diffH,
                VectorRotate.rotY(offsetL, 0, -Math.PI / 2),
                VectorRotate.rotY(offsetR, 0, -Math.PI / 2),
                VectorRotate.rotX(0, offsetH, -Math.PI / 2));

        double x = VectorRotate.rotX(displace[0], displace[1], Math.PI / 2);
        double y = VectorRotate.rotY(displace[0], displace[1], Math.PI / 2);
        displace[0] = x;
        displace[1] = y;

        currX += VectorRotate.rotX(displace[0], displace[1], currRot);
        currY += VectorRotate.rotY(displace[0], displace[1], currRot);
        currRot += displace[2];

        lastL = newL;
        lastR = newR;
        lastH = newH;

        tele.addData("encoder l", diffL);
        tele.addData("encoder r", diffR);
        tele.addData("encoder h", diffH);
        tele.addData("current x", currX);
        tele.addData("current y", currY);
        tele.addData("current rot", currRot);
    }

    public static double[] calcDisplace(double ax, double bx, double cy,
                                        double Ay, double By, double Cx) {

        double m = bx - ax;
        double M = Ay - By;
        double P = Ay * bx - By * ax;
        double Q = m * Cx - M * cy;

        double theta = m / M;

        double J = Math.sin(theta);
        double K = 1 - Math.cos(theta);
        double T = theta / 2 / K / M;

        double Fx = T * (J * P + K * Q);
        double Fy = T * (K * P - J * Q);

        return new double[] {Fx, Fy, theta};
    }
}
