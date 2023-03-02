package org.firstinspires.ftc.teamcode.hardware.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.util.VectorRotate;

// localization device
public class ConeDistLocalizer {

    // store telemetry device
    public Telemetry tele;

    // sensors
    public DistSensor sensor1;
    public DistSensor sensor2;

    // distance sensor locations
    public double offset1;
    public double offset2;

    // cone information
    public double coneRadius;
    public double viewRange;

    // detection result
    public boolean exist;
    public double locY;
    public double locX;

    // initialize device
    public ConeDistLocalizer(DistSensor sensor1, DistSensor sensor2,
                             double offset1, double offset2,
                             double coneRadius, double viewRange,
                             Telemetry tele) {

        this.tele = tele;

        this.sensor1 = sensor1;
        this.sensor2 = sensor2;

        this.offset1 = offset1;
        this.offset2 = offset2;

        this.coneRadius = coneRadius;
        this.viewRange = viewRange;
    }

    // localize cone by distance
    public void update(double currX, double currY, double currRot) {

        double circX1 = offset1;
        double circX2 = offset2;
        double circY1 = sensor1.dist();
        double circY2 = sensor2.dist();

        boolean exist1 = circY1 < viewRange;
        boolean exist2 = circY2 < viewRange;
        exist = exist1 || exist2;

        double globalX = (circX1 + circX2) / 2;
        double globalY = (circY1 + circY2) / 2;

        if (exist1 && exist2) {

            double dispX = circX2 - circX1;
            double dispY = circY2 - circY1;
            double disp = Math.sqrt(dispX * dispX + dispY * dispY);
            double dispTheta = Math.atan(dispY / dispX) + (dispX < 0 ? Math.PI : 0);

            double localX = disp / 2;
            double localY = coneRadius * coneRadius - localX * localX;
            if (localY < 0) localY = 0;
            else localY = Math.sqrt(localY);

            globalX = VectorRotate.rotX(localX, localY, dispTheta) + circX1;
            globalY = VectorRotate.rotY(localX, localY, dispTheta) + circY1;

        } else if (exist1) {

            globalX = circX1 - coneRadius;
            globalY = circY1;

        } else if (exist2) {

            globalX = circX2 + coneRadius;
            globalY = circY2;
        }

        locX = VectorRotate.rotX(globalX, globalY, currRot) + currX;
        locY = VectorRotate.rotY(globalX, globalY, currRot) + currY;

        tele.addData("cone position x", locX);
        tele.addData("cone position y", locY);
    }
}
