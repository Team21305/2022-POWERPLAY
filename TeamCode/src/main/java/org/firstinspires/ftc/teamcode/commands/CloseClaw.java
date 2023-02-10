package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubSystem;

public class CloseClaw extends CommandBase {

    private final IntakeSubSystem _intake;
    private final double _time;
    private long _startTime;


    public CloseClaw(IntakeSubSystem intake, double time){
        _intake = intake;
        _time = time;

        addRequirements(intake);
    }

    public void initialize() {
        _intake.close();
        _startTime = System.currentTimeMillis();
    }

    public void execute() {

    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return Math.abs(System.currentTimeMillis() - _startTime) > _time;
    }

}
