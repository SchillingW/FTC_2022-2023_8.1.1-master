package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RotateConvert;
import org.firstinspires.ftc.teamcode.util.VectorRotate;

// odometry device
public class HolonomicOdometry {

    // store telemetry device
    public Telemetry tele;

    // declare drive encoders
    public DcMotor encoderL;
    public DcMotor encoderR;
    public DcMotor encoderH;

    // declare drive encoder specifications
    public RotateConvert convertL;
    public RotateConvert convertR;
    public RotateConvert convertH;

    // encoder wheel offset from center of bot
    public double offsetL;
    public double offsetR;
    public double offsetH;

    // robot current location in global space
    public double currX;
    public double currY;
    public double currRot;

    // last recorded encoder outputs
    public double lastL;
    public double lastR;
    public double lastH;

    // initialize device
    public HolonomicOdometry(DcMotor encoderL, DcMotor encoderR, DcMotor encoderH,
                             RotateConvert convertL, RotateConvert convertR,
                             RotateConvert convertH,
                             double offsetL, double offsetR, double offsetH,
                             double startX, double startY, double startRot,
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

        this.currX = startX;
        this.currY = startY;
        this.currRot = startRot;

        this.lastL = encoderL.getCurrentPosition() / convertL.tickPerInch;
        this.lastR = encoderR.getCurrentPosition() / convertR.tickPerInch;
        this.lastH = encoderH.getCurrentPosition() / convertH.tickPerInch;
    }

    // track current robot location
    public void track() {

        // get encoder output
        double newL = encoderL.getCurrentPosition() / convertL.tickPerInch;
        double newR = encoderR.getCurrentPosition() / convertR.tickPerInch;
        double newH = encoderH.getCurrentPosition() / convertH.tickPerInch;

        // get encoder displacement
        double diffL = newL - lastL;
        double diffR = newR - lastR;
        double diffH = newH - lastH;

        // get global displacement by rotating encoder inputs to assumed horizontal space
        double[] displace = calcDisplace(
                VectorRotate.rotX(0, diffL, -Math.PI / 2),
                VectorRotate.rotX(0, diffR, -Math.PI / 2),
                VectorRotate.rotY(diffH, 0, -Math.PI / 2),
                VectorRotate.rotY(offsetL, 0, -Math.PI / 2),
                VectorRotate.rotY(offsetR, 0, -Math.PI / 2),
                VectorRotate.rotX(0, offsetH, -Math.PI / 2));

        // rotate result back to vertical space
        double x = VectorRotate.rotX(displace[0], displace[1], Math.PI / 2);
        double y = VectorRotate.rotY(displace[0], displace[1], Math.PI / 2);
        displace[0] = x;
        displace[1] = y;

        // rotate result to global space and apply
        currX += VectorRotate.rotX(displace[0], displace[1], currRot);
        currY += VectorRotate.rotY(displace[0], displace[1], currRot);
        currRot += displace[2];

        // store new encoder output
        lastL = newL;
        lastR = newR;
        lastH = newH;

        // telemetry debugging
        tele.addData("encoder l", diffL);
        tele.addData("encoder r", diffR);
        tele.addData("encoder h", diffH);
        tele.addData("current x", currX);
        tele.addData("current y", currY);
        tele.addData("current rot", currRot);
    }

    // calculate bot displacement by encoder displacement assuming initial horizontal orientation
    public static double[] calcDisplace(double ax, double bx, double cy,
                                        double Ay, double By, double Cx) {

        // compute constants on conclusion sheet
        double m = bx - ax;
        double M = Ay - By;
        double P = Ay * bx - By * ax;
        double Q = m * Cx - M * cy;

        // rotational offset
        double theta = m / M;

        // compute constants on conclusion sheet
        double J = Math.sin(theta);
        double K = 1 - Math.cos(theta);

        // linear offset
        double Fx;
        double Fy;

        // conclusion sheet breaks at 1-cos(theta)==0
        if (K == 0) {

            // use raw encoder input
            Fx = (ax + bx) / 2;
            Fy = cy;

        } else {

            // compute constants on conclusion sheet
            double T = theta / 2 / K / M;

            // use conclusion sheet
            Fx = T * (J * P + K * Q);
            Fy = T * (K * P - J * Q);
        }

        // return result as array
        return new double[] {Fx, Fy, theta};
    }
}
