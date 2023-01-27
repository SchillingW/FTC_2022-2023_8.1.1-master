package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Volta;
import org.firstinspires.ftc.teamcode.util.FieldDimensions;

// driver controlled opmode
@TeleOp(name="BasicDrive", group="LeaguePrep")
public class BasicDrive extends OpMode {

    // declare bot
    public Volta bot;

    // speed control
    public static final double turnSpeed = 0.6;
    public static final double speed0 = 0.4;
    public static final double speed1 = 0.7;
    public static final double speed2 = 1.0;
    public double speed = speed1;

    // slide control
    public boolean isAuto = true;

    @Override
    public void init() {

        // initialize bot
        bot = new Volta(0, 0, 0, hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        // reset gyro with button
        if (gamepad1.a) bot.nav.odometry.currRot = 0;

        // control speed with buttons
        if (gamepad1.x) speed = speed0;
        if (gamepad1.y) speed = speed1;
        if (gamepad1.b) speed = speed2;

        // control slide with buttons
        if (gamepad2.a) bot.slide.setTarget(Volta.restSlide);
        if (gamepad2.x) bot.slide.setTarget(FieldDimensions.lowGoal + Volta.aboveSlide);
        if (gamepad2.y) bot.slide.setTarget(FieldDimensions.midGoal + Volta.aboveSlide);
        if (gamepad2.b) bot.slide.setTarget(FieldDimensions.highGoal + Volta.aboveSlide);
        if (gamepad2.a || gamepad2.x || gamepad2.y || gamepad2.b) isAuto = true;

        // control claw with bumpers
        if (gamepad2.left_bumper) bot.claw.open();
        if (gamepad2.right_bumper) bot.claw.close();

        // get analog input
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rot = -gamepad1.right_stick_x;
        double slide = -gamepad2.right_stick_y;

        // readjust input magnitude
        x *= Math.sqrt(x * x + y * y);
        y *= Math.sqrt(x * x + y * y);
        rot *= Math.abs(rot);
        slide *= Math.abs(slide);

        // run drive train by input
        bot.nav.track();
        bot.nav.drive.run(x * speed, y * speed, rot * turnSpeed);

        // run linear slide to target
        if (slide != 0) {
            bot.slide.run(slide * bot.slide.velMag(slide));
            isAuto = false;
        } else if (isAuto) {
            bot.slide.update();
        } else {
            bot.slide.run(bot.slide.holdSpeed);
        }

        // telemetry debugging
        telemetry.addData("speed", speed);
        telemetry.update();
    }

    @Override
    public void stop() {

        // stop bot
        bot.stop();
    }
}
