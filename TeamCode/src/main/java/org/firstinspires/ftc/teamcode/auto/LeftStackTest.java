package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.bot.Volta;
import org.firstinspires.ftc.teamcode.util.FieldDimensions;

@Autonomous(name="LeftStackTest", group="LeaguePrep")
public class LeftStackTest extends OpMode {

    public Volta bot;

    @Override
    public void init() {

        bot = new Volta(FieldDimensions.cellSize + bot.frameX / 2, bot.frameY / 2, 0, hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        if (bot.next(bot.nav)) {
            bot.nav.setTarget(0, 0, 1.5 * FieldDimensions.cellSize, 0.5 * FieldDimensions.cellSize, 0);
            bot.slide.setTarget(bot.restSlide);
            bot.claw.close();
        }

        if (bot.next()) {
            bot.nav.setTarget(bot.clawX, bot.clawY, FieldDimensions.cellSize, 2 * FieldDimensions.cellSize, Math.PI / 2);
            bot.slide.setTarget(FieldDimensions.lowGoal + bot.aboveSlide);
        }

        if (bot.next(bot.slide)) {
            bot.slide.setTarget(FieldDimensions.lowGoal + bot.belowSlide);
            bot.claw.open();
        }

        if (bot.next(bot.nav)) {
            bot.nav.setTarget(0, 0, 1.5 * FieldDimensions.cellSize, 2.5 * FieldDimensions.cellSize, Math.PI / 2);
            bot.slide.setTarget(bot.restSlide);
        }

        if (bot.next()) {
            bot.nav.setTarget(bot.clawX, bot.clawY, FieldDimensions.stackX(0), FieldDimensions.stackY(0), Math.PI / 2);
            bot.slide.setTarget(FieldDimensions.stackHeight(5));
        }

        if (bot.next(bot.slide)) {
            bot.slide.setTarget(FieldDimensions.stackHeight(5) + bot.aboveSlide);
            bot.claw.close();
        }

        if (bot.next()) {
            bot.nav.setTarget(bot.clawX, bot.clawY, 2 * FieldDimensions.cellSize, 3 * FieldDimensions.cellSize, 0);
            bot.slide.setTarget(FieldDimensions.highGoal + bot.aboveSlide);
        }

        if (bot.next(bot.slide)) {
            bot.slide.setTarget(FieldDimensions.highGoal + bot.belowSlide);
            bot.claw.open();
        }

        if (bot.next()) {
            bot.nav.setTarget(0, 0, 1.5 * FieldDimensions.cellSize, 2.5 * FieldDimensions.cellCount, 0);
            bot.slide.setTarget(bot.startSlide);
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
