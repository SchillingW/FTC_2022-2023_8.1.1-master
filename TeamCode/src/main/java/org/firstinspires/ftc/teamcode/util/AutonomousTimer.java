package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.AutonomousSystem;

// timer device as autonomous timing system
public class AutonomousTimer extends AutonomousSystem {

    // declare timer device
    public ElapsedTime time;

    // declare target time
    public double target;

    // initialize device
    public AutonomousTimer() {

        time = new ElapsedTime();
        time.reset();
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
