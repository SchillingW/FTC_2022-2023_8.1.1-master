package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.bot.Volta;

@Autonomous(name="ParallelMovement", group="LeaguePrep")
public class ParallelMovement extends OpMode {

    public Volta bot;

    @Override
    public void init() {

        bot = new Volta(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        if (bot.next(bot.nav)) {
            bot.nav.setTarget(24, 0, Math.PI);
            bot.claw.open();
        }

        if (bot.next(bot.nav)) {
            bot.nav.setTarget(24, 24, 0);
        }

        if (bot.next(bot.slide)) {
            bot.slide.setTarget(36);
            bot.claw.close();
        }

        if (bot.next(bot.slide)) {
            bot.slide.setTarget(12);
            bot.claw.open();
        }

        if (bot.next()) {
            bot.nav.setTarget(-24, -24, 0);
            bot.slide.setTarget(24);
        }

        if (bot.next()) {
            bot.nav.setTarget(-12, -12, 0);
            bot.slide.setTarget(0);
            bot.claw.close();
        }

        if (bot.next()) {
            requestOpModeStop();
        }

        bot.update();
        telemetry.update();
    }

    @Override
    public void stop() {

        bot.stop();
    }
}
