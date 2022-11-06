package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
public class LiftSubsystem extends SubsystemBase {

    private final Hardware hardware;
    private final MultipleTelemetry telemetry;

    public static double bottom=0.96;
    public static double ground=0.92;
    public static double low=0.58;
    public static double middle=0.32;


    public LiftSubsystem(Hardware hardware, MultipleTelemetry telemetry) {

        this.hardware = hardware;
        this.telemetry = telemetry;
    }

    private void goToPosition(double position){
        hardware.liftServo0.setPosition(position);
        hardware.liftServo2.setPosition(position);
    }

    public void goToBottom(){
        goToPosition(bottom);
    }
    public void goToTop(){
        goToPosition(0);
}
    public void goToMiddle(){
        goToPosition(middle);
    }

    public void goToLow(){
        goToPosition(low);
    }

    public void goToGround(){
        goToPosition(ground);
    }

    @Override
    public void periodic() {
        super.periodic();

        telemetry.addData("bottomservo",bottom);

        telemetry.update();
    }
}







