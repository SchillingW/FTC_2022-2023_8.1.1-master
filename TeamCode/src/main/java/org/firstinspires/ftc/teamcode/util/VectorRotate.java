package org.firstinspires.ftc.teamcode.util;

// rotational vector math
public class VectorRotate {

    // x component of rotation
    public static double rotX(double x, double y, double rot) {

        return x * Math.cos(rot) - y * Math.sin(rot);
    }

    // y component of rotation
    public static double rotY(double x, double y, double rot) {

        return x * Math.sin(rot) + y * Math.cos(rot);
    }

    // x component of body such that local point lies over global point
    public static double anchoredX(double localX, double localY, double globalX, double rot) {

        return globalX - rotX(localX, localY, rot);
    }

    // y component of body such that local point lies over global point
    public static double anchoredY(double localX, double localY, double globalY, double rot) {

        return globalY - rotY(localX, localY, rot);
    }
}
