package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// claw device
public class Claw extends AutonomousSystem {

    // store telemetry device
    public Telemetry tele;

    // declare servo
    public Servo servo;

    // target positions
    public double closed;
    public double opened;

    // current state
    public String state = "neutral";

    // initialize device
    public Claw(Servo servo, double closed, double opened, Telemetry tele) {

        this.tele = tele;

        this.servo = servo;

        this.closed = closed;
        this.opened = opened;
    }

    // telemetry debugging
    @Override
    public void update() {

        super.update();
        tele.addData("claw", state);
    }

    // open claw
    public void open() {

        servo.setPosition(opened);
        state = "open";
    }

    // close claw
    public void close() {

        servo.setPosition(closed);
        state = "close";
    }
}
