package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bot.Volta;

// telemetry for simple field navigation
@TeleOp(name="Navigate", group="LeaguePrep")
public class Navigate extends OpMode {

    // declare bot
    public Volta bot;

    @Override
    public void init() {

        // initialize bot
        bot = new Volta(0, 0, 0, hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        if (gamepad1.a || gamepad2.a) {

            // return to origin on a press
            bot.update();

        } else {

            // track current position
            bot.track();

            // run drive train by input
            bot.nav.drive.run(
                    gamepad1.left_stick_x * 0.5,
                    -gamepad1.left_stick_y * 0.5,
                    -gamepad1.right_stick_x * 0.25,
                    bot.nav.odometry.currRot);

            // run linear slide by input
            bot.slide.run(-gamepad2.left_stick_y * 0.5);

            // telemetry debugging
            bot.claw.update();
            bot.timer.update();
        }

        // close claw on left bumper
        if (gamepad2.left_bumper) {

            bot.claw.close();
        }

        // open claw on right bumper
        if (gamepad2.right_bumper) {

            bot.claw.open();
        }

        // telemetry debugging
        telemetry.update();
    }

    @Override
    public void stop() {

        // stop bot
        bot.stop();
    }
}
