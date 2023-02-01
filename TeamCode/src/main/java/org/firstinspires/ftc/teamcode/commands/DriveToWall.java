package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveToWall extends CommandBase {

    private final DriveSubsystem drive;
    private final double distance;
    private final double speed;

    public DriveToWall(DriveSubsystem drive, double distance, double speed){

        this.drive = drive;
        this.distance = distance;
        this.speed = speed;

        addRequirements(drive);
    }

    public void initialize() {
    }

    public void execute() {
        drive.drive(speed, 0,0, false);
    }

    public void end(boolean interrupted) {
        drive.drive(0,0,0, false);
    }

    public boolean isFinished() {

        return drive.getRange()<= distance;

    }

}
