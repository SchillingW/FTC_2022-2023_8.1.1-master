package org.firstinspires.ftc.teamcode.util;

public class VectorRotate {

    public static double rotX(double x, double y, double rot) {

        return x * Math.cos(rot) - y * Math.sin(rot);
    }

    public static double rotY(double x, double y, double rot) {

        return x * Math.sin(rot) + y * Math.cos(rot);
    }
}
