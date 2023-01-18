package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HolonomicNavigation {

    public Telemetry tele;

    public HolonomicDrive drive;
    public HolonomicOdometry odometry;

    public HolonomicNavigation(HolonomicDrive drive, HolonomicOdometry odometry, Telemetry tele) {

        this.tele = tele;

        this.drive = drive;
        this.odometry = odometry;
    }
}
