package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// claw device
public class Claw {

    // store telemetry device
    public Telemetry tele;

    // declare servo
    public Servo servo;

    // target positions
    public double closed;
    public double opened;

    // initialize device
    public Claw(Servo servo, double closed, double opened, Telemetry tele) {

        this.tele = tele;

        this.servo = servo;

        this.closed = closed;
        this.opened = opened;
    }

    // open claw
    public void open() {

        servo.setPosition(opened);
    }

    // close claw
    public void close() {

        servo.setPosition(closed);
    }
}
