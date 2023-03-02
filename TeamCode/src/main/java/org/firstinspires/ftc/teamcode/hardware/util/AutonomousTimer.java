package org.firstinspires.ftc.teamcode.hardware.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.hardware.AutonomousSystem;

// timer device as autonomous timing system
public class AutonomousTimer extends AutonomousSystem {

    // store telemetry device
    public Telemetry tele;

    // declare timer device
    public ElapsedTime time;

    // declare target time
    public double target;

    // initialize device
    public AutonomousTimer(Telemetry tele) {

        this.tele = tele;

        time = new ElapsedTime();
        time.reset();
    }

    // telemetry debugging
    @Override
    public void update() {

        super.update();
        tele.addData("timer", time.seconds());
    }

    // return true if target time is passed
    @Override
    public boolean isDone() {

        boolean done = super.isDone();
        return done && isOver(target);
    }

    // return true if input target time is passed
    public boolean isOver(double target) {

        return time.seconds() >= target;
    }

    // set target time
    public void setTarget(double target) {

        this.target = target;
    }

    // reset timer at 0
    public void reset() {

        time.reset();
    }
}
