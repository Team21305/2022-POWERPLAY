package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@TeleOp
@Config
public class TrTest extends LinearOpMode {

    public static double F1 = 30;
    public static double T1 = 45;
    public static double F2 = 6;
    public static double F3 = 22.5;
    public static double T2 = 90;
    public static double F4 = 22.5;
    public static double B1 = 45.5;
    public static double T3 = 45;
    public static double F5 = 6;



    @Override
    public void runOpMode() throws InterruptedException {
        // Setup robot systems
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Hardware hardware = new Hardware(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LiftSubsystem lift = new LiftSubsystem(hardware, multipleTelemetry);
        IntakeSubSystem intake = new IntakeSubSystem(hardware, multipleTelemetry);

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
                .addTemporalMarker( ()->intake.close())
                .forward(F1)
                //raise lift after 5 inches
                .addTemporalMarker(2, ()->lift.goToLow())
                .turn(Math.toRadians(T1))
                .forward(F2)
                //Drop cone here
                .addTemporalMarker(()->intake.open())
                .addTemporalMarker(()->lift.goToPosition(0.18))
                .waitSeconds(0.5)
                .back(F2)
                .turn(Math.toRadians(-T1))
                .forward(F3)
                .turn(Math.toRadians(T2))
                .forward(F4)
                //pick up cone
                .addTemporalMarker(()->intake.close())
                .waitSeconds(0.5)
                .addTemporalMarker( ()->lift.goToLow())
                .waitSeconds(1)
                .back(B1)
                //lift to medium
                .addTemporalMarker(()->lift.goToMiddle())
                .turn(Math.toRadians(T3))
                .forward(F5)
                //Drop cone
                .addTemporalMarker(()->intake.open())
                .waitSeconds(0.5)
                .back(F5)
                .addTemporalMarker(()->lift.goToBottom())
                .turn(Math.toRadians(-T3))
                //Go get another cone
                .addTemporalMarker(()->lift.goToPosition(0.14))
                .forward(B1)
                .addTemporalMarker(()->intake.close())
                .waitSeconds(1)
                .addTemporalMarker( ()->lift.goToLow())
                .waitSeconds(1)
                .back(B1)
                .addTemporalMarker(()->lift.goToBottom())
                .waitSeconds(1)
                //0.14
                //park
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
