package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// distance sensor device
public class DistSensor {

    // store telemetry device
    public Telemetry tele;

    // declare sensor
    public ColorRangeSensor sensor;

    // offsets
    public static final double globalZero = 0;
    public double instanceZero;

    // initialize device
    public DistSensor(ColorRangeSensor sensor, double instanceZero, Telemetry tele) {

        this.tele = tele;

        this.sensor = sensor;

        this.instanceZero = instanceZero;
    }

    // get distance measurement
    public double dist() {

        double result = sensor.getDistance(DistanceUnit.INCH) - globalZero - instanceZero;
        tele.addData("distance", result);
        return result;
    }
}
