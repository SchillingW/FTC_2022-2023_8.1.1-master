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
            bot.nav.setTarget(48, 0, Math.PI);
        }

        if (bot.next(bot.nav)) {
            bot.nav.setTarget(48, 48, 0);
        }

        if (bot.next(bot.slide)) {
            bot.slide.setTarget(96);
        }

        if (bot.next(bot.slide)) {
            bot.slide.setTarget(24);
        }

        if (bot.next()) {
            bot.nav.setTarget(-48, -48, 0);
            bot.slide.setTarget(48);
        }

        if (bot.next()) {
            bot.nav.setTarget(0, 0, 0);
            bot.slide.setTarget(0);
        }

        bot.update();
        telemetry.update();
    }

    @Override
    public void stop() {

        bot.stop();
    }
}
