package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

import java.util.Arrays;

@Config
public class LiftSubsystem extends SubsystemBase {

    private final Hardware hardware;
    private final MultipleTelemetry telemetry;

    public static double bottom=0.02;
    public static double ground=0.04;
    public static double low=0.4;
    public static double middle=0.66;
    public static double top=1.0;
    public static double positionAdj = 0.02;

    private double currentPosition;

    private int sideCone = 0;

    public LiftSubsystem(Hardware hardware, MultipleTelemetry telemetry) {

        this.hardware = hardware;
        this.telemetry = telemetry;
    }

    public void goToPosition(double position){
        hardware.liftServo0.setPosition(position);
        hardware.liftServo2.setPosition(position);

        currentPosition = position;
    }

    public void sideStackUp(){
        if(sideCone >= 4){
            sideCone = 4;
        }
        else {
            sideCone = sideCone + 1;
        }

        goToPosition(0.02+sideCone*0.04);

    }

    public void sideStackDown(){
        if(sideCone <= 0){
            sideCone = 0;
        }
        else{
            sideCone = sideCone - 1;
        }

        goToPosition(0.02+sideCone*0.04);

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

    public static String[][] characters = {
            { "  ## ", " ######   ", " #####  ", " #   # ", " #####  "},
            { " ### ", "      ##  ", "     ## ", " #   # ", " #      "},
            { "  ## ", "  #####   ", "  ####  ", " ##### ", " #####  "},
            { "  ## ", " ##       ", "     ## ", "     # ", "     ## "},
            { "  ## ", " #######  ", " #####  ", "     # ", " #####  "}
    };

    public static String[] lines(int level){

        String[] lines = new String[6];
        Arrays.fill(lines, "");

        for(int row = 0; row < 5; row++){
            lines[row] += characters[row][level];
        }

        return lines;
    }

    @Override
    public void periodic() {

        telemetry.addData("currentPosition", currentPosition);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        for (String line : lines(sideCone)) {
            telemetry.addLine("<font face=\"monospace\">" + line.replaceAll("\\s", "&nbsp;") + "</font>");
        }

        telemetry.addData("^^ CONE STACK ^^", sideCone + 1);
    }
}







