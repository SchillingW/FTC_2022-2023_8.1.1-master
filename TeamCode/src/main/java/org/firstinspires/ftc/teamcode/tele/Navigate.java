package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Volta;

@TeleOp(name="Navigate", group="LeaguePrep")
public class Navigate extends OpMode {

    public Volta bot;

    @Override
    public void init() {

        bot = new Volta(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        bot.nav.drive.run(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
        bot.nav.odometry.update();

        telemetry.addData("L", bot.nav.odometry.lastL);
        telemetry.addData("R", bot.nav.odometry.lastR);
        telemetry.addData("H", bot.nav.odometry.lastH);

        telemetry.update();
    }
}
