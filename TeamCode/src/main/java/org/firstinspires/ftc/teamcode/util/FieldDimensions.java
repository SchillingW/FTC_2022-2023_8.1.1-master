package org.firstinspires.ftc.teamcode.util;

public class FieldDimensions {

    public static final double cellCount = 6;
    public static final double cellSize = 23.5;
    public static final double coneRadius = 2;

    public static final double groundGoal = 3;
    public static final double lowGoal = 12;
    public static final double midGoal = 24;
    public static final double highGoal = 36;

    public static final double coneGrab = 2.5;
    public static final double coneStack = 1;

    public static double stackX(double x) {

        return
                (1 - x) * coneRadius +
                x * (cellSize * cellCount - coneRadius);
    }

    public static double stackY(double y) {

        return
                (1 - y) * (cellSize * (cellCount / 2 - 0.5)) +
                y * (cellSize * (cellCount / 2 + 0.5));
    }

    public static double stackHeight(double count) {

        return coneStack * (count - 1) + coneGrab;
    }
}
