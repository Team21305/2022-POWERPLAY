package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

public class IntakeSubSystem extends SubsystemBase {


    private final Hardware hardware;
    private final Telemetry telemetry;

    public IntakeSubSystem(Hardware hardware, Telemetry telemetry) {

        this.hardware = hardware;
        this.telemetry = telemetry;
    }
    public void close(){
        hardware.liftServo4.setPosition(0);
        hardware.liftServo5.setPosition(0);
    }
    public void open(){
        hardware.liftServo4.setPosition(0.1);
        hardware.liftServo5.setPosition(0.1);
    }
}





