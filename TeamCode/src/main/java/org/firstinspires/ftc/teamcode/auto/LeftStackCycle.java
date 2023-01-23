package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.bot.Volta;
import org.firstinspires.ftc.teamcode.util.FieldDimensions;

// cycle cones from stack on high goal
@Autonomous(name="LeftStackCycle", group="LeaguePrep")
public class LeftStackCycle extends OpMode {

    // declare bot
    public Volta bot;

    @Override
    public void init() {

        // initialize bot
        bot = new Volta(
                2 * FieldDimensions.cellSize - FieldDimensions.cellMesh / 2 - Volta.wheelWidth - Volta.frameX / 2,
                FieldDimensions.cellMesh / 2 + Volta.frameY / 2,
                0,
                hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        // grab starting cone

        if (bot.next(bot.timer)) {
            bot.timer.setTarget(Volta.grabWait);
            bot.claw.close();
        }

        // navigate to high goal

        if (bot.next(bot.nav)) {
            bot.nav.setTarget(0, 0, 1.5 * FieldDimensions.cellSize, 2 * FieldDimensions.cellSize, 0);
            bot.slide.setTarget(Volta.restSlide);
        }

        if (bot.next(bot.nav, bot.slide)) {
            bot.nav.setTarget(Volta.clawX, Volta.clawY, 2 * FieldDimensions.cellSize, 3 * FieldDimensions.cellSize, 0);
            bot.slide.setTarget(FieldDimensions.highGoal + Volta.aboveSlide);
        }

        // drop on high goal

        if (bot.next(bot.timer)) {
            bot.timer.setTarget(Volta.dropBuffer);
            bot.slide.setTarget(FieldDimensions.highGoal + Volta.belowSlide);
        }

        if (bot.next(bot.slide)) {
            bot.claw.open();
        }

        // cycle from stack

        for (int i = 0; i < 3; i++) {

            // navigate to stack

            if (bot.next(bot.nav)) {
                bot.nav.setTarget(0, 0, 1.5 * FieldDimensions.cellSize, 2.5 * FieldDimensions.cellSize, Math.PI / 2);
                bot.slide.setTarget(Volta.restSlide);
            }

            if (bot.next(bot.nav, bot.slide)) {
                bot.nav.setTarget(Volta.clawX, Volta.clawY, FieldDimensions.stackX(0), FieldDimensions.stackY(0), Math.PI / 2);
                bot.slide.setTarget(FieldDimensions.stackHeight(5 - i));
            }

            // grab from stack

            if (bot.next(bot.timer)) {
                bot.timer.setTarget(Volta.grabWait);
                bot.claw.close();
            }

            if (bot.next(bot.slide)) {
                bot.slide.setTarget(FieldDimensions.stackHeight(5) + Volta.aboveSlide);
            }

            // navigate to high goal

            if (bot.next(bot.nav)) {
                bot.nav.setTarget(0, 0, 1.5 * FieldDimensions.cellSize, 2.5 * FieldDimensions.cellSize, Math.PI / 2);
                bot.slide.setTarget(Volta.restSlide);
            }

            if (bot.next(bot.nav, bot.slide)) {
                bot.nav.setTarget(Volta.clawX, Volta.clawY, 2 * FieldDimensions.cellSize, 3 * FieldDimensions.cellSize, 0);
                bot.slide.setTarget(FieldDimensions.highGoal + Volta.aboveSlide);
            }

            // drop on high goal

            if (bot.next(bot.timer)) {
                bot.timer.setTarget(Volta.dropBuffer);
                bot.slide.setTarget(FieldDimensions.highGoal + Volta.belowSlide);
            }

            if (bot.next(bot.slide)) {
                bot.claw.open();
            }
        }

        // park

        if (bot.next(bot.nav, bot.slide)) {
            bot.nav.setTarget(0, 0, 1.5 * FieldDimensions.cellSize, 2.25 * FieldDimensions.cellSize, 0);
            bot.slide.setTarget(Volta.startSlide);
        }

        // stop autonomous
        if (bot.next()) {
            requestOpModeStop();
        }

        // update bot
        if (bot.justInc) bot.timer.reset();
        bot.update();
        telemetry.update();
    }

    @Override
    public void stop() {

        // stop bot
        bot.stop();
    }
}
