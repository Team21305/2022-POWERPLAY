package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DriveStrafe extends CommandBase {

    private final DriveSubsystem drive;
    private final double distance;
    private final double speed;

    public DriveStrafe(DriveSubsystem drive, double distance, double speed){

        this.drive = drive;
        this.distance = distance;
        this.speed = speed;

        addRequirements(drive);
    }

    public void initialize() {
    }

    public void execute() {
        drive.drive(0,speed,0, false);
    }

    public void end(boolean interrupted) {
        drive.drive(0,0,0, false);
    }

    public boolean isFinished() {
        return Math.abs(drive.getPose().getY()) >= this.distance *1.35;

    }

}
