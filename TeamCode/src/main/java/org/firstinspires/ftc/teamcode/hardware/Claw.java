package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {

    public Telemetry tele;

    public Servo servo;

    public double closed;
    public double opened;

    public Claw(Servo servo, double closed, double opened, Telemetry tele) {

        this.tele = tele;

        this.servo = servo;

        this.closed = closed;
        this.opened = opened;
    }

    public void open() {

        servo.setPosition(opened);
    }

    public void close() {

        servo.setPosition(closed);
    }
}
