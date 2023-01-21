package org.firstinspires.ftc.teamcode.util;

public class InterpolateClamp {

    public final double minInput;
    public final double maxInput;
    public final double minOutput;
    public final double maxOutput;

    public InterpolateClamp(double minInput, double maxInput,
                            double minOutput, double maxOutput) {

        this.minInput = minInput;
        this.maxInput = maxInput;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    public double perform(double in) {

        double result =
                (in - minInput) / (maxInput - minInput) * (maxOutput - minOutput) + minOutput;

        return Math.min(Math.max(result, minOutput), maxOutput);
    }
}
