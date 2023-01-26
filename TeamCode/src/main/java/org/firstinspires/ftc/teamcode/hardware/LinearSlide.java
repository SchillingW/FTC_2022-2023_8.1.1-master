package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.InterpolateClamp;
import org.firstinspires.ftc.teamcode.util.RotateConvert;

// linear slide device
public class LinearSlide extends AutonomousSystem {

    // store telemetry device
    public Telemetry tele;

    // declare slide motor and specifications
    public DcMotor motor;
    public RotateConvert convert;

    // declare approach speed gradient
    public InterpolateClamp approachBelow;
    public InterpolateClamp approachAbove;

    // movement constants
    public double errorMargin;
    public double speedFactor;
    public double holdSpeed;

    // target finding
    public double target;
    public double curr;
    public double start;

    // declare devices
    public LinearSlide(
            DcMotor motor, RotateConvert convert,
            InterpolateClamp approachBelow, InterpolateClamp approachAbove,
            double errorMargin, double speedFactor, double holdSpeed, double start,
            Telemetry tele) {

        this.tele = tele;

        this.motor = motor;
        this.convert = convert;

        this.approachBelow = approachBelow;
        this.approachAbove = approachAbove;

        this.errorMargin = errorMargin;
        this.speedFactor = speedFactor;
        this.holdSpeed = holdSpeed;

        this.start = motor.getCurrentPosition() / convert.tickPerInch - start;

        // set target to current position
        setTarget(start);
    }

    // move towards target
    @Override
    public void update() {

        // update subsystems
        super.update();

        // track current position
        track();

        // hold position if reached
        if (isDone()) {

            run(holdSpeed);

        } else {

            // get displacement from target
            double diff = target - curr;

            // magnitude of vector towards target
            double currentMagnitude = Math.abs(diff);

            // desired magnitude by approach gradient
            double targetMagnitude = diff > 0 ?
                    approachBelow.perform(currentMagnitude) :
                    approachAbove.perform(currentMagnitude);

            // scale displacement to target magnitude
            diff *= targetMagnitude / currentMagnitude;

            // run device
            run(diff * speedFactor);
        }
    }

    // return true if at target
    @Override
    public boolean isDone() {

        // true if all subsystems are done
        boolean done = super.isDone();

        // true if current position in range of target
        return done && Math.abs(target - curr) <= errorMargin;
    }

    // set target height
    public void setTarget(double target) {

        this.target = target;
    }

    // run device at input speed
    public void run(double speed) {

        motor.setPower(speed * convert.polar);
        tele.addData("input slide", speed);
    }

    // get speed of slide based on move direction
    public double velMag(double dir) {

        return dir < 0 ? approachAbove.maxOutput : approachBelow.maxOutput;
    }

    // track device position
    public void track() {

        curr = motor.getCurrentPosition() / convert.tickPerInch - start;
        tele.addData("current slide", curr);
    }

    // stop linear slide
    public void stop() {

        run(0);
    }
}
