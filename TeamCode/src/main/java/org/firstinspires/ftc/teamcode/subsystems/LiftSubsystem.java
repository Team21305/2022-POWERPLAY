package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

public class LiftSubsystem extends SubsystemBase {

    private final Hardware hardware;
    private final Telemetry telemetry;

    public LiftSubsystem(Hardware hardware, Telemetry telemetry) {

        this.hardware = hardware;
        this.telemetry = telemetry;
    }

    public void goToBottom(){
        hardware.liftServo0.setPosition(0);
        hardware.liftServo2.setPosition(0);
    }
    public void goToTop(){
        hardware.liftServo0.setPosition(1);
        hardware.liftServo2.setPosition(1);
}
    public void goToMiddle(){
        hardware.liftServo0.setPosition(0.5);
        hardware.liftServo2.setPosition(0.5);
    }
}







