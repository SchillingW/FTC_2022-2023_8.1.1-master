package org.firstinspires.ftc.teamcode.hardware.hardware;

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
    public double offset;

    // initialize device
    public DistSensor(ColorRangeSensor sensor, double offset, Telemetry tele) {

        this.tele = tele;

        this.sensor = sensor;

        this.offset = offset;
    }

    // get distance measurement
    public double dist() {

        double result = sensor.getDistance(DistanceUnit.INCH) - offset;
        tele.addData("distance", result);
        return result;
    }
}
