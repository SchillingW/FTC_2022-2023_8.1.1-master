package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSlideClaw extends DriveSlide {

    public Claw claw;

    public DriveSlideClaw(HolonomicNavigation nav, LinearSlide slide, Claw claw, Telemetry tele) {

        super(nav, slide, tele);

        this.claw = claw;
    }
}
