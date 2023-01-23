package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.bot.Volta;

// autonomous test for movement in parallel
@Autonomous(name="ParallelMovement", group="LeaguePrep")
public class ParallelMovement extends OpMode {

    // declare bot
    public Volta bot;

    @Override
    public void init() {

        // initialize bot
        bot = new Volta(0, 0, 0, hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        // move right and rotate
        if (bot.next(bot.nav)) {
            bot.nav.setTarget(24, 0, Math.PI);
            bot.slide.setTarget(Volta.restSlide);
            bot.claw.open();
        }

        // move forward and rotate back
        if (bot.next(bot.nav)) {
            bot.nav.setTarget(24, 24, 0);
        }

        // raise slide and close claw
        if (bot.next(bot.slide)) {
            bot.slide.setTarget(36);
            bot.claw.close();
        }

        // lower slide and open claw
        if (bot.next(bot.slide)) {
            bot.slide.setTarget(12);
            bot.claw.open();
        }

        // raise slide and move back left
        if (bot.next(bot.nav, bot.slide)) {
            bot.nav.setTarget(-24, -24, 0);
            bot.slide.setTarget(24);
        }

        // lower slide and move to origin
        if (bot.next(bot.nav, bot.slide)) {
            bot.nav.setTarget(-12, -12, 0);
            bot.slide.setTarget(Volta.restSlide);
            bot.claw.close();
        }

        // stop autonomous
        if (bot.next()) {
            requestOpModeStop();
        }

        // update bot
        bot.update();
        telemetry.update();
    }

    @Override
    public void stop() {

        // stop bot
        bot.stop();
    }
}
