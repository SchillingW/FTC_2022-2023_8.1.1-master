package org.firstinspires.ftc.teamcode.hardware.prototype;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.util.VectorRotate;

@TeleOp(name="DistanceDemo",group="LeaguePrep")
public class DistanceDemo extends OpMode {

    public DistanceSensor distance1;
    public DistanceSensor distance2;

    @Override
    public void init() {

        distance1 = hardwareMap.get(DistanceSensor.class, "sensor1");
        distance2 = hardwareMap.get(DistanceSensor.class, "sensor2");
    }

    @Override
    public void loop() {

        double dist1 = distance1.getDistance(DistanceUnit.INCH);
        double dist2 = distance2.getDistance(DistanceUnit.INCH);
        double[] pos = getPos(dist1, dist2, 1.3, 1.6);
        telemetry.addData("1", String.format("%.2f", dist1));
        telemetry.addData("2", String.format("%.2f", dist2));
        telemetry.addData("X", String.format("%.2f", pos[0]));
        telemetry.addData("Y", String.format("%.2f", pos[1]));
    }

    public double[] getPos(double dist1, double dist2, double offset, double radius) {

        double x1 = -offset / 2;
        double y1 = dist1;
        double x2 = offset / 2;
        double y2 = dist2;

        double dispX = x2 - x1;
        double dispY = y2 - y1;
        double theta = Math.atan(dispY / dispX) + (dispX < 0 ? Math.PI : 0);
        double d = Math.sqrt(dispX * dispX + dispY * dispY);

        double localX = d / 2;
        double localY = Math.sqrt(radius * radius - localX * localX);

        double middleX = VectorRotate.rotX(localX, localY, theta);
        double middleY = VectorRotate.rotY(localX, localY, theta);

        double finalX = middleX + x1;
        double finalY = middleY + y1;

        return new double[] {finalX, finalY};
    }
}
