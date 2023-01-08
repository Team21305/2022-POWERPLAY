package org.firstinspires.ftc.teamcode.autonomous.opmode.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubSystem;

public abstract class AutoBase extends LinearOpMode {

    protected LiftSubsystem lift;
    protected IntakeSubSystem intake;
    protected SampleMecanumDrive drive;
    //protected VisionSubSystem vision;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup robot systems
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Hardware hardware = new Hardware(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        lift = new LiftSubsystem(hardware, multipleTelemetry);
        intake = new IntakeSubSystem(hardware, multipleTelemetry);
        //vision = new VisionSubSystem(hardware, multipleTelemetry);

        // Update drive with our starting pose.
        drive.setPoseEstimate(getStartingPose());

        TrajectorySequence auto = getSequence();

        waitForStart();


        TrajectorySequence park = getParkingTrajectory(auto.end());

        if (!isStopRequested()) {
            drive.followTrajectorySequence(auto);
            if(park != null) {
                drive.setPoseEstimate(auto.end());
                drive.followTrajectorySequence(park);
            }
        }
    }

    protected abstract TrajectorySequence getSequence();

    protected abstract Pose2d getStartingPose();

    protected abstract TrajectorySequence getParkingTrajectory(Pose2d start);
}
