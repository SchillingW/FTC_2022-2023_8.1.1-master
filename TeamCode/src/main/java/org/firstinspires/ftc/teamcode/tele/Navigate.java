package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Volta;

@TeleOp(name="Navigate", group="LeaguePrep")
public class Navigate extends OpMode {

    public Volta bot;

    @Override
    public void init() {

        bot = new Volta(0, 0, 0, hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        if (gamepad1.a || gamepad2.a) {

            bot.update();

        } else {

            bot.track();

            bot.nav.drive.run(
                    gamepad1.left_stick_x * 0.5,
                    -gamepad1.left_stick_y * 0.5,
                    -gamepad1.right_stick_x * 0.25,
                    bot.nav.odometry.currRot);

            bot.slide.run(-gamepad2.left_stick_y * 0.5);
        }

        if (gamepad2.left_bumper) {

            bot.claw.close();
        }

        if (gamepad2.right_bumper) {

            bot.claw.open();
        }

        telemetry.update();
    }

    @Override
    public void stop() {

        bot.stop();
    }
}
