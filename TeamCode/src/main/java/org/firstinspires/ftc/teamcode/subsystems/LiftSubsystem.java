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

    public static double bottom=0.02;
    public static double ground=0.04;
    public static double low=0.4;
    public static double middle=0.64;
    public static double top=1.0;
    public static double positionAdj = 0.02;

    private double currentPosition;

    public LiftSubsystem(Hardware hardware, MultipleTelemetry telemetry) {

        this.hardware = hardware;
        this.telemetry = telemetry;
    }

    private void goToPosition(double position){
        hardware.liftServo0.setPosition(position);
        hardware.liftServo2.setPosition(position);

        currentPosition = position;
    }

    public void bumpUp(){
        double pos = currentPosition+positionAdj;

        if(pos > 1){
            pos = 1;
        }

        goToPosition(pos);
    }

    public void bumpDown(){
        double pos = currentPosition-positionAdj;

        if(pos < 0){
            pos = 0;
        }

        goToPosition(pos);
    }

    public void goToBottom(){
        goToPosition(bottom);
    }
    public void goToTop(){
        goToPosition(top);
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

    public double getPosition(){
        return currentPosition;
    }

    @Override
    public void periodic() {
        super.periodic();

        telemetry.addData("currentPosition",currentPosition);

        telemetry.update();
    }
}







