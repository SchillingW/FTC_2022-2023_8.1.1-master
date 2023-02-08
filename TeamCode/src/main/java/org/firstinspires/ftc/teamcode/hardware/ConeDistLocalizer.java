package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.VectorRotate;

// localization device
public class ConeDistLocalizer {

    // store telemetry device
    public Telemetry tele;

    // sensors
    public int sensorCount;
    public DistSensor[] sensor;

    // distance sensor locations
    public double[] offset;

    // cone information
    public double coneRadius;
    public double viewRange;

    // detection result
    public boolean exist;
    public double locY;
    public double locX;

    // initialize device
    public ConeDistLocalizer(int sensorCount, DistSensor[] sensor,
                             double[] offset, double coneRadius, double viewRange,
                             Telemetry tele) {

        this.tele = tele;

        this.sensorCount = sensorCount;
        this.sensor = sensor;

        this.offset = offset;
        this.coneRadius = coneRadius;
        this.viewRange = viewRange;
    }

    // localize cone by distance
    public void update(double currX, double currY, double currRot) {

        double[] input = new double[sensorCount];
        for (int i = 0; i < sensorCount; i++) input[i] = sensor[i].dist();

        exist = false;
        int min = -1;
        for (int i = 0; i < sensorCount - 1; i++) {
            boolean thisExist = input[i] < viewRange && input[i + 1] < viewRange;
            exist = exist || thisExist;
            boolean thisMin = min == -1 || input[i] + input[i + 1] < input[min] + input[min + 1];
            if (thisExist && thisMin) min = i;
        }

        double circX1 = offset[min];
        double circX2 = offset[min + 1];
        double circY1 = input[min];
        double circY2 = input[min + 1];

        exist = circY1 < viewRange && circY2 < viewRange;

        double dispX = circX2 - circX1;
        double dispY = circY2 - circY1;
        double disp = Math.sqrt(dispX * dispX + dispY * dispY);
        double dispTheta = Math.atan(dispY / dispX) + (dispX < 0 ? Math.PI : 0);

        double localX = disp / 2;
        double localY = coneRadius * coneRadius - localX * localX;
        if (localY < 0) localY = 0;
        else localY = Math.sqrt(localY);

        double globalX = VectorRotate.rotX(localX, localY, dispTheta) + circX1;
        double globalY = VectorRotate.rotY(localX, localY, dispTheta) + circY1;

        locX = VectorRotate.rotX(globalX, globalY, currRot) + currX;
        locY = VectorRotate.rotY(globalX, globalY, currRot) + currY;

        tele.addData("cone position x", locX);
        tele.addData("cone position y", locY);
    }
}
