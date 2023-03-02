package org.firstinspires.ftc.teamcode.OLDPROJECT.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.OLDPROJECT.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.OLDPROJECT.botconfigs.PursuitBotTesting;
import org.firstinspires.ftc.teamcode.OLDPROJECT.hardware.VisionDevice;

@Autonomous(name="29_LOW_LEFT", group="PursuitBot")
public class Meet4Auto_LEFT extends LinearOpMode {

    public double poleCellDiff;
    public PursuitBotTesting robot;
    public VisionDevice vision;
    public LinearSlide linearSlide;
    ColorSensor sensorColor;
    public int dropOffset = 40;
    public ColorSensor sensor;

    public boolean moveToNext;

    //auto
    @Override
    public void runOpMode() {
        robot = new PursuitBotTesting(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 0.125;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2;
        sensor = hardwareMap.colorSensor.get("sensor");
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 1;

        poleCellDiff = robot.xDim.toPole(1) - robot.xDim.toCell(1) + 0.5;

        vision = new VisionDevice(telemetry, hardwareMap);
        vision.init();

        linearSlide = new LinearSlide(telemetry, hardwareMap);
        sleep(1000);
        int result = 0;
        while (!isStarted()) {
            int next = vision.perform(1f / 3f);
            if (next != -1) result = next;
            telemetry.addData("current result", result);
            telemetry.update();
        }
        // START MOVEMENT
        waitForStart();
        if (opModeIsActive()) linearSlide.closeClaw();
        sleep(300);

        // CONE GRABBED
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(1), new Rotation2d()), telemetry, this, linearSlide, linearSlide.driveHeight, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1), new Rotation2d()), telemetry, this, linearSlide, linearSlide.low, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(1.5) , robot.yDim.toCell(1) - poleCellDiff, Rotation2d.fromDegrees(-90)), telemetry, this, linearSlide, linearSlide.low, true);
        linearSlide.goToFull(linearSlide.low + 75, telemetry, this);
        if (opModeIsActive()) linearSlide.openClaw();
        sleep(200);

        Cycle(0);
        Cycle(1);

        if(result == 1)
        {
            Cycle(2);
            robot.reachPointSlide(new Pose2d(robot.xDim.toCell(1.5) , robot.yDim.toCell(1) + 2, Rotation2d.fromDegrees(-90)), telemetry, this, linearSlide, linearSlide.low, true);
        }

        else
        {
            robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1) - poleCellDiff, Rotation2d.fromDegrees(-90)), telemetry, this, false);
            if(result == 0) robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(0), Rotation2d.fromDegrees(-90)), telemetry, this, linearSlide, linearSlide.stacks[3], true);
            else robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(result) + 2, Rotation2d.fromDegrees(-90)), telemetry, this, linearSlide, linearSlide.ground, true);
        }
    }

    public void Cycle(int i)
    {
        double inc = 3;
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1) - poleCellDiff, Rotation2d.fromDegrees(-90 - i * inc)), telemetry, this, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(0) - 1.325, Rotation2d.fromDegrees(-90 - i * inc)), telemetry, this, linearSlide, linearSlide.stacks[i], true);
        linearSlide.closeClaw();
        sleep(300);
        linearSlide.goTo(linearSlide.low - 50, telemetry);
        sleep(150);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1) - poleCellDiff, Rotation2d.fromDegrees(-90 - i * inc)), telemetry, this, linearSlide, linearSlide.low - 50, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(1.5) , robot.yDim.toCell(1) - poleCellDiff, Rotation2d.fromDegrees(-90 - i * inc)), telemetry, this, linearSlide, linearSlide.low, true);
        linearSlide.goToFull(linearSlide.low + 75, telemetry, this);
        if (opModeIsActive()) linearSlide.openClaw();
        sleep(200);
    }
}

