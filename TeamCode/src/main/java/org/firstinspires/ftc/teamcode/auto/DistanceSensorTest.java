package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bot.Volta;

@Autonomous(name="DistanceSensorTest", group="LeaguePrep")
public class DistanceSensorTest extends OpMode {

    public Volta bot;
    public ColorSensor sensor;
    public DistanceSensor distance;
    RevBlinkinLedDriver lights;
    @Override
    public void init() {

        bot = new Volta(0, 0, 0, hardwareMap, telemetry);
        sensor = hardwareMap.colorSensor.get("sensor");
        distance = hardwareMap.get(DistanceSensor.class, "sensor");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
    }

    @Override
    public void loop() {

        /*if (bot.next(bot.nav)) {
            bot.nav.setTarget(24, 0, Math.PI);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }*/
        telemetry.addData("distance", distance.getDistance(DistanceUnit.INCH));
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        telemetry.update();


        /*if (distance.getDistance(DistanceUnit.INCH) ) {
            bot.nav.setTarget(24, 0, Math.PI);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            bot.slide.setTarget(bot.restSlide);
        }
        else{
            bot.claw.open();
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        }*/

        bot.update();
        telemetry.update();
    }

    @Override
    public void stop() {

        bot.stop();
    }
}
