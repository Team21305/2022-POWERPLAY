package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@TeleOp
public class TrTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup robot systems
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Hardware hardware = new Hardware(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LiftSubsystem lift = new LiftSubsystem(hardware, multipleTelemetry);


        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive
                .trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(10, 10), 0)
                .turn(Math.toRadians(90))
                .splineTo(new Vector2d(25, -15), 0)
                .waitSeconds(3)
                .turn(Math.toRadians(45))
                .forward(10)
                .strafeRight(5)
                .turn(Math.toRadians(90))
                .strafeLeft(5)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(45)), 0)
                .build();

        TrajectorySequence auto = drive
                .trajectorySequenceBuilder(startPose)
                .forward(18)
                .turn(Math.toRadians(45))
                .waitSeconds(2)
                .turn(Math.toRadians(-45))
                .forward(24)
                .turn(Math.toRadians(90))
                .forward(24)
                .waitSeconds(2)
                .forward(-48)
                .build();

        TrajectorySequence norahDrive = drive
                .trajectorySequenceBuilder(startPose)
                .forward(35)
                .strafeRight(35)
                .back(35)
                .strafeLeft(35)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(auto);
    }

}
