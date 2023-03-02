package org.firstinspires.ftc.teamcode.hardware.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// drive train with odometry and linear slide bot
public class DriveSlide extends AutonomousSystem {

    // store telemetry device
    public Telemetry tele;

    // declare subsystems
    public HolonomicNavigation nav;
    public LinearSlide slide;

    // initialize device
    public DriveSlide(HolonomicNavigation nav, LinearSlide slide, Telemetry tele) {

        this.tele = tele;

        this.nav = nav;
        this.slide = slide;

        // add autonomous subsystems
        subsystem.add(nav);
        subsystem.add(slide);
    }

    // track current position of subsystems
    public void track() {

        nav.track();
        slide.track();
    }

    // stop all subsystems
    public void stop() {

        nav.stop();
        slide.stop();
    }
}
