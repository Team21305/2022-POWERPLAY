package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class DefaultDrive extends CommandBase {

    private final DriveSubsystem driveSubsysem;
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
    public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        driveSubsysem = subsystem;
        m_forward = forward;
        m_strafe = strafe;
        m_rotation = rotation;
        addRequirements(driveSubsysem);
    }

    @Override
    public void execute() {
        driveSubsysem.drive(m_forward.getAsDouble()*0.75, m_strafe.getAsDouble()*0.75, m_rotation.getAsDouble()*0.75);
    }

}
