package org.firstinspires.ftc.teamcode.util;

public class RotateConvert {

    public final int polar;
    public final int tickPerRev;
    public final double inchPerRev;
    public final double tickPerInch;

    public RotateConvert(int tickPerRev, double inchPerRev, int polar) {

        this.polar = polar;
        this.tickPerRev = tickPerRev * polar;
        this.inchPerRev = inchPerRev;
        this.tickPerInch = tickPerRev / inchPerRev * polar;
    }

    public RotateConvert(int tickPerRev, double inchPerRev) {

        this(tickPerRev, inchPerRev, 1);
    }

    public RotateConvert instance(int polar) {

        return new RotateConvert(tickPerRev, inchPerRev, polar);
    }
}
