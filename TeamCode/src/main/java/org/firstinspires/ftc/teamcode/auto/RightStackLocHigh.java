package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.bot.Volta;
import org.firstinspires.ftc.teamcode.util.FieldDimensions;

// cycle cones from stack on high goal
@Autonomous(name="RightStackLocHigh", group="LeaguePrep")
public class RightStackLocHigh extends OpMode {

    // declare bot
    public Volta bot;

    @Override
    public void init() {

        // initialize bot
        bot = new Volta(
                (FieldDimensions.cellCount - 2) * FieldDimensions.cellSize + FieldDimensions.cellMesh / 2 + Volta.wheelWidth + Volta.frameX / 2 + 1.8,
                FieldDimensions.cellMesh / 2 + Volta.frameY / 2,
                0,
                hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {

        // update vision
        bot.vision.loop();

        // telemetry debugging
        telemetry.update();
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
            bot.nav.setTarget(0, 0, (FieldDimensions.cellCount - 1.5) * FieldDimensions.cellSize + 2, 2.75 * FieldDimensions.cellSize, 0);
            bot.slide.setTarget(Volta.restSlide + Volta.aboveSlide);
        }

        if (bot.next(bot.nav, bot.slide)) {
            bot.nav.setTarget(Volta.clawX, Volta.clawY, (FieldDimensions.cellCount - 2) * FieldDimensions.cellSize, 3 * FieldDimensions.cellSize + 2, 0);
            bot.slide.setTarget(FieldDimensions.highGoal + Volta.aboveSlide);
        }

        if (bot.next(bot.timer)) {
            bot.timer.setTarget(Volta.alignTime);
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

        for (int i = 1; i <= 3; i++) {

            // navigate to stack

            if (bot.next(bot.nav)) {
                bot.nav.setTarget(0, 0, (FieldDimensions.cellCount - 1.5) * FieldDimensions.cellSize, 2.5 * FieldDimensions.cellSize, -Math.PI / 2);
            }

            if (bot.next(bot.nav, bot.slide)) {
                bot.nav.approach = Volta.approachStack;
                bot.nav.setTarget(Volta.clawX, Volta.clawY, FieldDimensions.stackX(1) - 1, FieldDimensions.stackY(0) - 2, -Math.PI / 2, bot.coneLoc);
                bot.slide.setTarget(FieldDimensions.stackHeight(6 - i));
            }

            if (bot.next(bot.timer)) {
                bot.nav.setTarget(-Math.PI / 2, bot.coneLoc);
                bot.timer.setTarget(Volta.alignTime);
            }

            if (bot.end()) {
                bot.nav.approach = Volta.approachFast;
                bot.nav.relocalize();
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
                bot.nav.setTarget(0, 0, (FieldDimensions.cellCount - 1.5) * FieldDimensions.cellSize, 2.5 * FieldDimensions.cellSize, -Math.PI / 2);
            }

            if (bot.next(bot.nav, bot.slide)) {
                bot.nav.approach = Volta.approachPole;
                bot.nav.setTarget(Volta.clawX, Volta.clawY, (FieldDimensions.cellCount - 2) * FieldDimensions.cellSize, 3 * FieldDimensions.cellSize, 0, bot.coneLoc);
                bot.slide.setTarget(FieldDimensions.highGoal + Volta.aboveSlide);
            }

            if (bot.next(bot.timer)) {
                bot.nav.setTarget(0, bot.coneLoc);
                bot.timer.setTarget(Volta.alignTime);
            }

            if (bot.end()) {
                bot.nav.approach = Volta.approachFast;
                bot.nav.relocalize();
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
            bot.nav.setTarget(0, 0, (FieldDimensions.cellCount - 2.5 + bot.vision.result) * FieldDimensions.cellSize, 2.5 * FieldDimensions.cellSize, -Math.PI / 2);
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
