package org.firstinspires.ftc.teamcode.hardware.util;

public class InterpolateClamp {

    // define curve
    public final double minInput;
    public final double maxInput;
    public final double minOutput;
    public final double maxOutput;

    // initialize curve
    public InterpolateClamp(double minInput, double maxInput,
                            double minOutput, double maxOutput) {

        this.minInput = minInput;
        this.maxInput = maxInput;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    // transform input along curve
    public double perform(double in) {

        double result =
                (in - minInput) / (maxInput - minInput) * (maxOutput - minOutput) + minOutput;

        return Math.min(Math.max(result, minOutput), maxOutput);
    }
}
