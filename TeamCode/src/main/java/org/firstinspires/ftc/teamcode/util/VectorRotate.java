package org.firstinspires.ftc.teamcode.util;

public class VectorRotate {

    public static double rotX(double x, double y, double rot) {

        return x * Math.cos(rot) - y * Math.sin(rot);
    }

    public static double rotY(double x, double y, double rot) {

        return x * Math.sin(rot) + y * Math.cos(rot);
    }

    public static double anchoredX(double localX, double localY, double globalX, double rot) {

        return globalX - rotX(localX, localY, rot);
    }

    public static double anchoredY(double localX, double localY, double globalY, double rot) {

        return globalY - rotY(localX, localY, rot);
    }
}
