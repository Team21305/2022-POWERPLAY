package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * @author Jackson
 * @see Path
 */
public class PurePursuit extends CommandBase {

    private MecanumDrive m_drive;
    private DriveSubsystem m_odometry;
    private Path m_path;

    public PurePursuit(MecanumDrive drive, DriveSubsystem odometry, Waypoint... waypoints) {
        m_path = new Path(waypoints);
        m_drive = drive;
        m_odometry = odometry;

        addRequirements(odometry);
    }

    @Override
    public void initialize() {
        m_path.init();
    }

    public void addWaypoint(Waypoint waypoint) {
        m_path.add(waypoint);
    }

    public void addWaypoints(Waypoint... waypoints) {
        for (Waypoint waypoint : waypoints) this.addWaypoint(waypoint);
    }

    public void removeWaypointAtIndex(int index) {
        m_path.remove(index);
    }

    /**
     * Call this in a loop
     */
    @Override
    public void execute() {
        Pose2d robotPose = m_odometry.getPose();
        double[] motorSpeeds = m_path.loop(robotPose.getTranslation().getX(), robotPose.getTranslation().getY(), robotPose.getHeading());
        m_drive.driveRobotCentric(motorSpeeds[0], motorSpeeds[1], motorSpeeds[2]);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return m_path.isFinished();
    }

}



