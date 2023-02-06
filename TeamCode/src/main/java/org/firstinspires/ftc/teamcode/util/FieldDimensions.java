package org.firstinspires.ftc.teamcode.util;

// field measurement definition
public class FieldDimensions {

    // grid size
    public static final double cellCount = 6;
    public static final double cellSize = 23.5;
    public static final double cellMesh = 1.8;

    // junction size
    public static final double groundGoal = 1;
    public static final double lowGoal = 13.5;
    public static final double midGoal = 23.5;
    public static final double highGoal = 33.5;

    // cone size
    public static final double coneRadius = 2;
    public static final double coneGrab = 2.5;
    public static final double coneStack = 1.2;

    // hardware size
    public static final double sensorWidth = 0.7;

    // get position of indexed stack in x axis
    public static double stackX(int x) {

        return new InterpolateClamp(
                0, 1,
                cellMesh / 2 + coneRadius,
                cellSize * cellCount - cellMesh / 2 - coneRadius
        ).perform(x);
    }

    // get position of indexed stack in y axis
    public static double stackY(int y) {

        return new InterpolateClamp(
                0, 1,
                cellSize * (cellCount / 2 - 0.5),
                cellSize * (cellCount / 2 + 0.5)
        ).perform(y);
    }

    // get target grab height of stack
    public static double stackHeight(double count) {

        return coneStack * (count - 1) + coneGrab;
    }
}
