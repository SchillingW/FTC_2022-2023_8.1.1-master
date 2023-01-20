package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSlide extends AutonomousSystem {

    public Telemetry tele;

    public HolonomicNavigation nav;
    public LinearSlide slide;

    public DriveSlide(HolonomicNavigation nav, LinearSlide slide, Telemetry tele) {

        this.tele = tele;

        this.nav = nav;
        this.slide = slide;

        subsystem.add(nav);
        subsystem.add(slide);
    }

    public void track() {

        nav.track();
        slide.track();
    }

    public void stop() {

        nav.stop();
        slide.stop();
    }
}
