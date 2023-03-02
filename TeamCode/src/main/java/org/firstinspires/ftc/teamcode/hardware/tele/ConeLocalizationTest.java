package org.firstinspires.ftc.teamcode.hardware.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.bot.Volta;

// telemetry for simple field navigation
@TeleOp(name="ConeLocalizationTest", group="LeaguePrep")
public class ConeLocalizationTest extends OpMode {

    // declare bot
    public Volta bot;

    @Override
    public void init() {

        // initialize bot
        bot = new Volta(0, 0, 0, hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        bot.coneLoc.update(bot.nav.odometry.currX, bot.nav.odometry.currY, bot.nav.odometry.currRot);
        if (bot.coneLoc.exist) {
            bot.nav.setTarget(Volta.clawX, Volta.clawY, bot.coneLoc.locX, bot.coneLoc.locY, bot.nav.odometry.currRot);
        } else {
            bot.nav.setTarget(0, 0, 0);
        }
        bot.update();
    }

    @Override
    public void stop() {

        // stop bot
        bot.stop();
    }
}
