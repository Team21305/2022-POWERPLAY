package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

import java.util.Vector;

@Disabled
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
    private LiftSubsystem lift;
    private IntakeSubSystem intake;

    //public static double PARK_POS = 22;
    //public static double PARK_POS = 0;
    public static double PARK_POS = -24;


    @Override
    public void runOpMode() throws InterruptedException {
        // Setup robot systems
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Hardware hardware = new Hardware(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        lift = new LiftSubsystem(hardware, multipleTelemetry);
        intake = new IntakeSubSystem(hardware, multipleTelemetry);

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
                .addTemporalMarker(()->intake.close())
                .forward(F1)
                //raise lift after 2 seconds
                .addTemporalMarker(2, ()->lift.goToLow())
                .turn(Math.toRadians(T1))
                .forward(F2)
                //Drop cone here
                .addTemporalMarker(()->intake.open())
                //Set lift height for top cone of cone stack
                .addTemporalMarker(()->lift.goToPosition(0.18))
                .waitSeconds(0.5)
                .back(F2)
                .turn(Math.toRadians(-T1))
                .forward(F3)
                .turn(Math.toRadians(T2))
                .forward(F4)
                //pick up cone
//                .closeClawAndLift(lift, intake)
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
//                .closeClawAndLift(lift, intake)
                .back(B1)
                .addTemporalMarker(()->lift.goToBottom())
                .waitSeconds(1)
                //0.14
                //park
                .build();

        TrajectorySequence poseTest = drive
                .trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->lift.goToBottom())
                .addTemporalMarker(()->intake.close())
                .addTemporalMarker(1, ()->lift.goToLow())
                //.lineTo(new Vector2d(30, 0))
                .splineTo(new Vector2d(33, 5), Math.toRadians(45))
                // drop cone
                .addTemporalMarker(()->intake.open())
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(57, 0, Math.toRadians(90)))

                // drive to cone stack
                .addTemporalMarker(()->lift.goToPosition(0.18))
                .lineTo(new Vector2d(55, 22))

                // pickup top cone
                .addTemporalMarker(()->intake.close())
                .waitSeconds(0.4)
                .addTemporalMarker( ()->lift.goToLow())
                .waitSeconds(0.4)

                // go to medium junction
                .addTemporalMarker(()->lift.goToMiddle())
                .lineToLinearHeading(new Pose2d(55, -17, Math.toRadians(180)))
                .lineTo(new Vector2d(51, -17))


                // drop cone
                .addTemporalMarker(()->intake.open())
                .waitSeconds(0.5)

                .setReversed(true)
                .lineTo(new Vector2d(55, -17))
                .lineToLinearHeading(new Pose2d(55, 22, Math.toRadians(90)))
                .addTemporalMarker(()->lift.goToPosition(0.14))

                .build();

    double multiplier = 1.0;

        TrajectorySequence poseTest2 = drive
                .trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->intake.close())
                .addTemporalMarker(.3, ()->lift.goToLow())
                //.lineTo(new Vector2d(30, 0))
                .splineTo(new Vector2d(33, multiplier * 5), Math.toRadians(multiplier * 45))
                // drop cone
                .addTemporalMarker(()->intake.open())
                //.waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(57, 0, Math.toRadians(multiplier * 90)))

                // drive to cone stack
                .addTemporalMarker(()->lift.goToPosition(0.18))
                .splineTo(new Vector2d(49, multiplier * 22), Math.toRadians(multiplier * 130))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(57, 0, Math.toRadians(multiplier * 90)), Math.toRadians(0))
                .setReversed(false)
                .lineTo(new Vector2d(55, multiplier * 22))

                // pickup top cone
                .addTemporalMarker(()->intake.close())
                .waitSeconds(0.4)
                //.addTemporalMarker( ()->lift.goToLow())
                //.waitSeconds(0.5)

                // go to medium juntion
                .addTemporalMarker(()->lift.goToMiddle())
                .lineToLinearHeading(new Pose2d(55, multiplier * -13.5, Math.toRadians(multiplier * 90)))
                .turn(Math.toRadians(multiplier * 90))

                // drop cone
                .addTemporalMarker(()->intake.open())
                //.waitSeconds(0.5)

                // undo turn
                .turn(Math.toRadians(multiplier * -90))

                .addTemporalMarker(()->lift.goToPosition(0.14))
                .lineToLinearHeading(new Pose2d(55, multiplier * 22, Math.toRadians(multiplier * 90)))


                // pickup cone
                .addTemporalMarker(()->intake.close())
                .waitSeconds(0.4)
                //.addTemporalMarker( ()->lift.goToLow())
                //.waitSeconds(0.5)

                // go to medium junction
                .addTemporalMarker(()->lift.goToMiddle())
                .lineToLinearHeading(new Pose2d(55, multiplier * -13.5, Math.toRadians(multiplier * 90)))
                .turn(Math.toRadians(multiplier * 90))

                // drop cone
                .addTemporalMarker(()->intake.open())
                .waitSeconds(0.5)

                // turn and go back to stack
                .turn(Math.toRadians(-90))
                .addTemporalMarker(()->lift.goToBottom())

                //Park
                .lineToLinearHeading(new Pose2d(55, PARK_POS, Math.toRadians(90)))
                .turn(Math.toRadians(-90))


                .build();

        Pose2d startPose2 = new Pose2d(0, 0, Math.toRadians(90));

        drive.setPoseEstimate(startPose2);

        // Vectors
        double flipOverX = 1;
        double medJunXOffset = 0;

        Vector2d coneStackPre = new Vector2d(-56, -12 * flipOverX);
        Vector2d coneStack = new Vector2d(-58.5, -12 * flipOverX);
        Vector2d terminal = new Vector2d(-56, -60 * flipOverX);
        Vector2d lowJunction = new Vector2d(-46, -12 * flipOverX);
        Vector2d mediumJunction = new Vector2d(-22+medJunXOffset, -12 * flipOverX);
        Vector2d readSignal = new Vector2d(-30, -60* flipOverX);

        Vector2d park = new Vector2d(-12, -12 * flipOverX);

        double right90 = Math.toRadians(-90 * flipOverX);
        double left90 = Math.toRadians(90 * flipOverX);
        Pose2d startPose3 = new Pose2d(-31, -63 * flipOverX, Math.toRadians(90 * flipOverX));

        drive.setPoseEstimate(startPose3);

        // written from Red Left
        TrajectorySequence poseTest3 = drive
                .trajectorySequenceBuilder(startPose3)

                // read signal
                .lineToLinearHeading(new Pose2d(readSignal, Math.toRadians(90 * flipOverX)))
                .waitSeconds(.1)

                // To Terminal
                .turn(left90)
                .lineToLinearHeading(new Pose2d(terminal, Math.toRadians(180)))
                // drop cone
                .addTemporalMarker(()->intake.open())
                .waitSeconds(.2)
                .turn(right90)

                // To Cone Stack
                .lineToLinearHeading(new Pose2d(coneStackPre, Math.toRadians(90 * flipOverX)))
                .addTemporalMarker(()->lift.goToPosition(0.18))
                .turn(left90)
                .lineToLinearHeading(new Pose2d(coneStack, Math.toRadians(180)))

                // grab cone
                .addTemporalMarker(()->intake.close())
                .waitSeconds(0.4)
                .addTemporalMarker( ()->lift.goToLow())

                // To Low Junction
                .lineToLinearHeading(new Pose2d(lowJunction, Math.toRadians(180)))
                .turn(left90)
                // Drop Cone
                .addTemporalMarker(()->intake.open())


                // To Cone Stack
                .turn(right90)
                .addTemporalMarker(()->lift.goToPosition(0.14))
                .lineToLinearHeading(new Pose2d(coneStack, Math.toRadians(180)))
                // grab cone
                .addTemporalMarker(()->intake.close())
                .waitSeconds(0.4)
                .addTemporalMarker( ()->lift.goToLow())

                // To Medium Junction
                .lineToLinearHeading(new Pose2d(mediumJunction, Math.toRadians(180)))
                .addTemporalMarker(()->lift.goToMiddle())
                .turn(left90)
                // Drop Cone
                .addTemporalMarker(()->intake.open())
                .waitSeconds(.2)

                // Park
                .lineToLinearHeading(new Pose2d(park, Math.toRadians(90)))
                //.turn(right90)
                .waitSeconds(.2)

                .build();


        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(poseTest3);
    }

}
