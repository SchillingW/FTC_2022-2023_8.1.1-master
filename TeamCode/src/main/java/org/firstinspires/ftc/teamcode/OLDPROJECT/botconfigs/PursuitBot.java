package org.firstinspires.ftc.teamcode.OLDPROJECT.botconfigs;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OLDPROJECT.FieldDimensions;
import org.firstinspires.ftc.teamcode.OLDPROJECT.hardware.MecDriveFlip;

import java.util.function.DoubleSupplier;

// mecanum drive bot with odometry for Pure Pursuit
public class PursuitBot {

    // dimensions
    public FieldDimensions xDim = new FieldDimensions();
    public FieldDimensions yDim = new FieldDimensions();

    // debugging devices
    public Telemetry tele;

    public ColorSensor sensor;
    public int dropOffset = 80;

    // mecanum wheel drive train
    public MecanumDrive drive;
    public Motor motorFL;
    public Motor motorFR;
    public Motor motorBL;
    public Motor motorBR;
    public LinearSlide linearSlide;

    // odometry device
    public OdometrySubsystem odometry;
    public DoubleSupplier encoderL;
    public DoubleSupplier encoderR;
    public DoubleSupplier encoderH;

    // hardware specifications
    public double wheelDiameter = 2.3622;
    public double wheelCircumference = wheelDiameter * Math.PI;

    // robot type data
    public double encoderTrackWidth = 8.5;
    public double encoderWheelOffset = 1.5;

    // robot movement datas
    public double adjustSpeed = 0.15;
    public double minSpeed = 0.3;
    public double minGradient = 3;
    public double maxSpeed = 0.6;
    public double maxGradient = 18;
    public double errorMargin = 0.5;
    public double extraTime = 0.5;
    public double degreeToInchEquivFactor = 24.0 / 360.0;

    public double rotErrorMargin = 3;

    // initialize devices
    public PursuitBot(Telemetry tele, HardwareMap map) {

        // store debugging device
        this.tele = tele;

        // initialize dimensions hi
        xDim.botanchor2botcenterHARDWARE = 6.5;
        yDim.botanchor2botcenterHARDWARE = 6.5;
        xDim.botanchor2clawcenterHARDWARE = 18.5;
        yDim.botanchor2clawcenterHARDWARE = 6.5;

        // initialize drive train
        motorFL = new Motor(map, "motorFL");
        motorFR = new Motor(map, "motorFR");
        motorBL = new Motor(map, "motorBL");
        motorBR = new Motor(map, "motorBR");
        linearSlide = new LinearSlide(tele, map);

        //sensor = map.colorSensor.get("sensor");

        drive = new MecDriveFlip(
                motorFL, motorFR, motorBL, motorBR,
                1, 1, 1, 1);

        // initialize odometry hi
        encoderL = getSupplier(motorFL, 1);
        encoderR = getSupplier(motorFR, -1);
        encoderH = getSupplier(motorBL, 1);
        odometry = new OdometrySubsystem(new HolonomicOdometry(
                encoderL, encoderR, encoderH,
                encoderTrackWidth, encoderWheelOffset));

        // orient to home
        motorFL.resetEncoder();
        motorFR.resetEncoder();
        motorBL.resetEncoder();
        motorBR.resetEncoder();
    }

    // return double supplier representing motor value in inches
    public DoubleSupplier getSupplier(Motor encoder, float coefficient) {

        // convert motor ticks to inches
        double ticksPerInch = 8192 / wheelCircumference;
        return () -> encoder.getCurrentPosition() / ticksPerInch * coefficient;
    }

    public double inchesToMeters(double inches)
    {
        return inches * 0.0254;
    }



    public void reachPoint(Pose2d target, Telemetry tele, LinearOpMode mode) {
        //Optional <String> l = Optional.ofNullable(level);

        if (mode.opModeIsActive()) {

            odometry.update();

            while (!isAtTarget(target) && mode.opModeIsActive()) {

                odometry.update();
                moveTowards(true, target, tele);
            }

            ElapsedTime time = new ElapsedTime();

            while (time.seconds() < extraTime && mode.opModeIsActive()) {

                odometry.update();
                moveTowards(false, target, tele);
            }

            drive.stop();
        }
    }

    /*public void reachPoint(Pose2d target, Telemetry tele, LinearOpMode mode, String level) {
        //Optional <String> l = Optional.ofNullable(level);

        if (mode.opModeIsActive()) {

            odometry.update();

            while (!isAtTarget(target) && mode.opModeIsActive()) {
                if (level == "low") {
                    linearSlide.goTo(linearSlide.low + dropOffset, telemetry);
                }
                if (level == "medium") {
                    linearSlide.goTo(linearSlide.med, telemetry);
                }
                if (level == "high") {
                    linearSlide.goTo(linearSlide.high, telemetry);
                }
                odometry.update();
                moveTowards(true, target, tele);
            }

            ElapsedTime time = new ElapsedTime();

            while (time.seconds() < extraTime && mode.opModeIsActive()) {

                odometry.update();
                moveTowards(false, target, tele);
            }

            drive.stop();
        }
    }*/

    public void moveTowards(boolean largeMove, Pose2d target, Telemetry tele) {

        double x = target.getX() - odometry.getPose().getX();
        double y = target.getY() - odometry.getPose().getY();
        double rot = target.getRotation().minus(odometry.getPose().getRotation()).getDegrees() * degreeToInchEquivFactor;
        double currentMagnitude = Math.sqrt(x * x + y * y + rot * rot);

        double x1 = minGradient; double x2 = maxGradient; double y1 = minSpeed; double y2 = maxSpeed;
        double targetMagnitude = (y1 - y2) / (x1 - x2) * (currentMagnitude - x1) + y1;
        targetMagnitude = largeMove ? Math.min(Math.max(targetMagnitude, y1), y2) : adjustSpeed;

        tele.addData("current magnitude", currentMagnitude);
        tele.addData("target magnitude", targetMagnitude);
        tele.addData("x", x);
        tele.addData("y", y);
        tele.addData("rot", rot);

        x *= targetMagnitude / currentMagnitude;
        y *= targetMagnitude / currentMagnitude;
        rot *= targetMagnitude / currentMagnitude;

        drive.driveFieldCentric(x, y, rot, odometry.getPose().getHeading());

        DebugFull(tele);
    }

    public boolean isAtTarget(Pose2d target) {

        double x = target.getX() - odometry.getPose().getX();
        double y = target.getY() - odometry.getPose().getY();
        double rot = target.getRotation().minus(odometry.getPose().getRotation()).getDegrees() * degreeToInchEquivFactor;

        return Math.abs(x) <= errorMargin && Math.abs(y) <= errorMargin && Math.abs(rot) <= errorMargin;
    }

    public void TranslateY(double y, double speed, Telemetry tele, LinearOpMode mode)
    {
        double initialY = odometry.getPose().getY();
        double targetY = initialY + y;
        double currentY = initialY;

        while(!doneTranslating(initialY, currentY, targetY) && mode.opModeIsActive())
        {
            tele.addData("translating", 0);
            drive.driveWithMotorPowers(speed, speed, speed, speed);
            currentY = odometry.getPose().getY();
            odometry.update();
        }

        drive.stop();
    }

    public void TranslateX(double x, double speed, Telemetry tele, LinearOpMode mode)
    {
        double initialX = odometry.getPose().getX();
        double targetX = initialX + x;
        double currentX = initialX;

        while(!doneTranslating(initialX, currentX, targetX) && mode.opModeIsActive())
        {
            tele.addData("translating", 0);
            drive.driveWithMotorPowers(speed, speed, speed, speed);
            currentX = odometry.getPose().getX();
            odometry.update();
        }

        drive.stop();
    }

    public boolean doneTranslating(double initial, double current, double target)
    {
        return (initial < target) ? current >= target : current <= target;
    }


    public void CellToStackLeft(double speed, Telemetry tele, LinearOpMode mode, ColorSensor sensor)
    {
        double currentDegrees = odometry.getPose().getRotation().getDegrees();
        double targetDegrees = -75;

        while(currentDegrees > targetDegrees && mode.opModeIsActive())
        {
            if(sensor.blue() > 245 || sensor.red() > 245) break;
            drive.driveFieldCentric(0, 0, -1 * speed, odometry.getPose().getHeading());
            currentDegrees = odometry.getPose().getRotation().getDegrees();
            odometry.update();
        }

        drive.stop();
    }

    public void CellToStackRight(double speed, Telemetry tele, LinearOpMode mode, ColorSensor sensor)
    {
        double currentDegrees = odometry.getPose().getRotation().getDegrees();
        double targetDegrees = 75;

        while(currentDegrees < targetDegrees && mode.opModeIsActive())
        {
            if(sensor.blue() > 245 || sensor.red() > 245) break;
            drive.driveFieldCentric(0, 0, speed, odometry.getPose().getHeading());
            currentDegrees = odometry.getPose().getRotation().getDegrees();
            odometry.update();
        }

        drive.stop();
    }

    public void GroundToLow(double speed, Telemetry tele, LinearOpMode mode, ColorSensor sensor)
    {
        double currentDegrees = odometry.getPose().getRotation().getDegrees();
        double targetDegrees = -157.5;

        while(currentDegrees > targetDegrees && mode.opModeIsActive())
        {
            //if(sensor.blue() > 250) break;
            drive.driveFieldCentric(0, 0, -1 * speed, odometry.getPose().getHeading());
            currentDegrees = odometry.getPose().getRotation().getDegrees();
            odometry.update();
        }

        drive.stop();
    }

    // debug info on bot with telemetry
    public void DebugFull(Telemetry telemetry) {

        telemetry.addData("current pose", odometry.getPose());
        telemetry.addData("encoder vertical left", encoderL.getAsDouble());
        telemetry.addData("encoder vertical right", encoderR.getAsDouble());
        telemetry.addData("encoder horizontal", encoderH.getAsDouble());
        telemetry.addData("blue", sensor.blue());
        telemetry.update();
    }


    //EESH NEW METHODS
    public void Translate(Translation2d reach, boolean forward)
    {
        double tol = 0.2;
        double changePos = 3;
        Translation2d pointA, pointB;
        double xDistance, yDistance;

        //Define the two points
        if(forward)
        {
            pointA = new Translation2d(odometry.getPose().getX(),odometry.getPose().getY());
            pointB = new Translation2d(reach.getX(), reach.getY());
        }

        else
        {
            pointA = new Translation2d(reach.getX(), reach.getY());
            pointB = new Translation2d(odometry.getPose().getX(),odometry.getPose().getY());
        }


        //Create variables to store the distance between the two points
        if(forward)
        {
            xDistance = pointB.getX() - pointA.getX();
            yDistance = pointB.getY() - pointA.getY();
        }

        else
        {
            xDistance = pointB.getY() - pointA.getY();
            yDistance = pointB.getX() - pointA.getX();
        }

        //xDistance = Math.min(1, (Math.max(-1, xDistance)));
        //yDistance = Math.min(1, (Math.max(-1, yDistance)));

        //Create a variable to store the total distance between the two points
        double totalDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        //Create a variable to store the speed of the robot
        double speed = 1;

        //Loop until the robot reaches the destination
        while (totalDistance > tol) {
            //Move the robot
            drive.driveRobotCentric(xDistance * speed, yDistance * speed, 0);
            odometry.update();
            if(forward) pointA = new Translation2d(odometry.getPose().getX(),odometry.getPose().getY());
            else pointB = new Translation2d(odometry.getPose().getX(),odometry.getPose().getY());

            //Calculate the new distance between the two points
            if(forward)
            {
                xDistance = pointB.getX() - pointA.getX();
                yDistance = pointB.getY() - pointA.getY();
            }

            else
            {
                xDistance = pointB.getY() - pointA.getY();
                yDistance = pointB.getX() - pointA.getX();
            }

            //xDistance = Math.min(1, (Math.max(-1, xDistance)));
            //yDistance = Math.min(1, (Math.max(-1, yDistance)));

            totalDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

            //Slow the robot as it approaches the destination
            speed = Math.max(speed - 0.1, 0.15);
            if(totalDistance <= tol) break;
        }

        drive.stop();
    }

    public void Rotate(double reachDegrees)
    {
        double tol = 0.5;

        //Define the starting degrees
        double startDegrees = odometry.getPose().getRotation().getDegrees();

        //Create variables to store the distance between the two points
        double distance = reachDegrees - startDegrees;
        //distance = Math.min(1, (Math.max(-1, distance)));

        //Create a variable to store the speed of the robot
        double speed = 1;

        //Loop until the robot reaches the destination
        while (distance > tol) {
            //Move the robot
            drive.driveRobotCentric(0, 0, distance * speed);

            //Calculate the new distance between the two
            double currentDegrees = odometry.getPose().getRotation().getDegrees();
            distance = reachDegrees - currentDegrees;
            //distance = Math.min(1, (Math.max(-1, distance)));

            //Slow the robot as it approaches the destination
            speed = Math.max(speed - 0.1, 0.1);
        }

        drive.stop();
    }
}
//changes