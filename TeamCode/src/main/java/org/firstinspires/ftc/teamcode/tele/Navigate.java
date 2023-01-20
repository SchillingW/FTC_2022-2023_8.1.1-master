package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Volta;

@TeleOp(name="Navigate", group="LeaguePrep")
public class Navigate extends OpMode {

    public Volta bot;

    @Override
    public void init() {

        bot = new Volta(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        if (gamepad1.a) {

            bot.update();

        } else {

            bot.nav.drive.run(
                    gamepad1.left_stick_x * 0.5,
                    -gamepad1.left_stick_y * 0.5,
                    -gamepad1.right_stick_x * 0.25,
                    bot.nav.odometry.currRot);

            bot.slide.run(-gamepad2.left_stick_y * 0.5);

            bot.track();
        }

        telemetry.update();
    }

    @Override
    public void stop() {

        bot.stop();
    }
}
