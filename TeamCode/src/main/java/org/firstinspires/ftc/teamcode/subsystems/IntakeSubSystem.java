package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
public class IntakeSubSystem extends SubsystemBase {

    public static double OPEN_POSITION = 0.1;
    public static double CLOSE_POSITION = 0;

    public static double OPEN_POSITION2 = 0.2;
    public static double CLOSE_POSITION2 = 0.1;

    private final Hardware hardware;
    private final MultipleTelemetry telemetry;

    public IntakeSubSystem(Hardware hardware, MultipleTelemetry telemetry) {

        this.hardware = hardware;
        this.telemetry = telemetry;
    }
    public void close(){
        hardware.liftServo4.setPosition(CLOSE_POSITION2);
        hardware.liftServo5.setPosition(CLOSE_POSITION);
    }
    public void open(){
        hardware.liftServo4.setPosition(OPEN_POSITION2);
        hardware.liftServo5.setPosition(OPEN_POSITION);
    }
}





