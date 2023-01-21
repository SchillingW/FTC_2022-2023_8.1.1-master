package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.AutonomousSystem;

public class AutonomousTimer extends AutonomousSystem {

    public ElapsedTime time;

    public double target;

    public AutonomousTimer() {

        time = new ElapsedTime();
        time.reset();
    }

    @Override
    public boolean isDone() {

        boolean done = super.isDone();
        return done && isOver(target);
    }

    public boolean isOver(double target) {

        return time.seconds() >= target;
    }

    public void setTarget(double target) {

        this.target = target;
    }

    public void reset() {

        time.reset();
    }
}
