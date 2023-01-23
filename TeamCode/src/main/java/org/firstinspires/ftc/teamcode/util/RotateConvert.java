package org.firstinspires.ftc.teamcode.util;

// convert measurements for rotating hardware device
public class RotateConvert {

    // -1 or 1 for forward direction
    public final int polar;

    // conversion factors
    public final int tickPerRev;
    public final double inchPerRev;
    public final double tickPerInch;

    // initialize device
    public RotateConvert(int tickPerRev, double inchPerRev, int polar) {

        this.polar = polar;
        this.tickPerRev = tickPerRev * polar;
        this.inchPerRev = inchPerRev;
        this.tickPerInch = tickPerRev / inchPerRev * polar;
    }

    // initialize device with default polarity
    public RotateConvert(int tickPerRev, double inchPerRev) {

        this(tickPerRev, inchPerRev, 1);
    }

    // create copy of device with new polarity
    public RotateConvert instance(int polar) {

        return new RotateConvert(tickPerRev, inchPerRev, polar);
    }
}
