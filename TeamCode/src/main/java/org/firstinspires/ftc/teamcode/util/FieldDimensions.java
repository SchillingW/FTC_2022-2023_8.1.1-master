package org.firstinspires.ftc.teamcode.util;

// field measurement definition
public class FieldDimensions {

    // grid size
    public static final double cellCount = 6;
    public static final double cellSize = 23.5;
    public static final double cellMesh = 1.8;

    // junction size
    public static final double groundGoal = 1;
    public static final double lowGoal = 13;
    public static final double midGoal = 23;
    public static final double highGoal = 33;

    // cone size
    public static final double coneRadius = 2;
    public static final double coneGrab = 2.5;
    public static final double coneStack = 1.19;

    // get position of indexed stack in x axis
    public static double stackX(double x) {

        return
                (1 - x) * coneRadius +
                x * (cellSize * cellCount - coneRadius);
    }

    // get position of indexed stack in y axis
    public static double stackY(double y) {

        return
                (1 - y) * (cellSize * (cellCount / 2 - 0.5)) +
                y * (cellSize * (cellCount / 2 + 0.5));
    }

    // get target grab height of stack
    public static double stackHeight(double count) {

        return coneStack * (count - 1) + coneGrab;
    }
}
