package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class DefaultDrive extends CommandBase {

    private final DriveSubsystem driveSubsysem;
    private final LiftSubsystem liftSubsystem;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_strafe;
    private final DoubleSupplier m_rotation;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param rotation  The control input for turning
     */
    public DefaultDrive(DriveSubsystem subsystem, LiftSubsystem liftSubsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        driveSubsysem = subsystem;
        this.liftSubsystem = liftSubsystem;
        m_forward = forward;
        m_strafe = strafe;
        m_rotation = rotation;
        addRequirements(driveSubsysem);
    }

    @Override
    public void execute() {

        double liftPosition = liftSubsystem.getPosition();

        double scale = 1.0;
        double scale2 = 1.0;


        if (liftPosition > 0.7){

            scale = (-0.5/0.3) * liftPosition + 2.166;
            scale2 = 1 - scale * 0.5;

        }

        driveSubsysem.drive(scale * m_forward.getAsDouble(),scale * m_strafe.getAsDouble(), scale2 * m_rotation.getAsDouble());
    }

}
