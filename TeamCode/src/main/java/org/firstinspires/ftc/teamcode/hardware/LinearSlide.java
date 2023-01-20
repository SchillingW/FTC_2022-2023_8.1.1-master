package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.InterpolateClamp;
import org.firstinspires.ftc.teamcode.util.RotateConvert;

public class LinearSlide extends AutonomousSystem {

    public Telemetry tele;

    public DcMotor motor;
    public RotateConvert convert;

    public InterpolateClamp approachBelow;
    public InterpolateClamp approachAbove;

    public double errorMargin;
    public double speedFactor;
    public double target;
    public double curr;

    public LinearSlide(
            DcMotor motor, RotateConvert convert,
            InterpolateClamp approachBelow, InterpolateClamp approachAbove,
            double errorMargin, double speedFactor,
            Telemetry tele) {

        this.tele = tele;

        this.motor = motor;
        this.convert = convert;

        this.approachBelow = approachBelow;
        this.approachAbove = approachAbove;

        this.errorMargin = errorMargin;
        this.speedFactor = speedFactor;
    }

    @Override
    public void update() {

        super.update();

        track();

        double diff = target - curr;

        double currentMagnitude = Math.abs(diff);

        double targetMagnitude = diff > 0 ?
                approachBelow.perform(currentMagnitude) :
                approachAbove.perform(currentMagnitude);

        diff *= targetMagnitude / currentMagnitude;

        run(diff * speedFactor);
    }

    @Override
    public boolean isDone() {

        boolean done = super.isDone();

        return done && Math.abs(target - curr) <= errorMargin;
    }

    public void setTarget(double target) {

        this.target = target;
    }

    public void run(double speed) {

        motor.setPower(speed * convert.polar);
        tele.addData("input slide", speed);
    }

    public void track() {

        curr = motor.getCurrentPosition() / convert.tickPerInch;
        tele.addData("current slide", curr);
    }

    public void stop() {

        run(0);
    }
}
