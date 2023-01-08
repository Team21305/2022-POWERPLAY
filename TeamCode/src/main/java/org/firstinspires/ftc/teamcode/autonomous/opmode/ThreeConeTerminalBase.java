package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.autonomous.opmode.core.AutoBase;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;

public class ThreeConeTerminalBase extends AutoBase {

    private double flipOverX;
    Vector2d coneStackPre;
    Vector2d coneStack;
    Vector2d terminal;
    Vector2d lowJunction;
    Vector2d mediumJunction;
    Vector2d readSignal;

    Vector2d park;

    double right90;
    double left90;
    Pose2d startPose;

    ThreeConeTerminalBase(Boolean isQuad2){

        flipOverX = isQuad2 ? -1 : 1;

        coneStackPre = new Vector2d(-56, -12 * flipOverX);
        coneStack = new Vector2d(-58.5, -12 * flipOverX);
        terminal = new Vector2d(-56, -60 * flipOverX);
        lowJunction = new Vector2d(-46, -12 * flipOverX);
        mediumJunction = new Vector2d(-22+medJunXOffset, -12 * flipOverX);
        readSignal = new Vector2d(-30.5, -60* flipOverX);

        park = new Vector2d(-12, -12 * flipOverX);

        right90 = Math.toRadians(-90 * flipOverX);
        left90 = Math.toRadians(90 * flipOverX);
        startPose = new Pose2d(-30.5, -64.5 * flipOverX, Math.toRadians(90 * flipOverX));
    }

    double medJunXOffset = 0;

    @Override
    protected TrajectorySequence getSequence() {

        return drive
            .trajectorySequenceBuilder(startPose)

            // read signal
            .lineToLinearHeading(new Pose2d(readSignal, Math.toRadians(90 * flipOverX)))
            .waitSeconds(.1)

            // To Terminal
            .turn(left90)
            .lineToLinearHeading(new Pose2d(terminal, Math.toRadians(180)))
            // drop cone
            .addTemporalMarker(() -> intake.open())
            .waitSeconds(.2)
            .turn(right90)

            // To Cone Stack
            .lineToLinearHeading(new Pose2d(coneStackPre, Math.toRadians(90 * flipOverX)))
            .addTemporalMarker(() -> lift.goToPosition(0.18))
            .turn(left90)
            .lineToLinearHeading(new Pose2d(coneStack, Math.toRadians(180)))

            // grab cone
            .addTemporalMarker(() -> intake.close())
            .waitSeconds(0.4)
            .addTemporalMarker(() -> lift.goToLow())

            // To Low Junction
            .lineToLinearHeading(new Pose2d(lowJunction, Math.toRadians(180)))
            .turn(left90)
            // Drop Cone
            .addTemporalMarker(() -> intake.open())


            // To Cone Stack
            .turn(right90)
            .addTemporalMarker(() -> lift.goToPosition(0.14))
            .lineToLinearHeading(new Pose2d(coneStack, Math.toRadians(180)))
            // grab cone
            .addTemporalMarker(() -> intake.close())
            .waitSeconds(0.4)
            .addTemporalMarker(() -> lift.goToLow())

            // To Medium Junction
            .lineToLinearHeading(new Pose2d(mediumJunction, Math.toRadians(180)))
            .addTemporalMarker(() -> lift.goToMiddle())
            .turn(left90)
            // Drop Cone
            .addTemporalMarker(() -> intake.open())
            .waitSeconds(.2)


            .build();
    }

    @Override
    protected TrajectorySequence getParkingTrajectory(Pose2d beginPose) {

        //int tag = vision.getTag();

        return drive
                .trajectorySequenceBuilder(beginPose)

                // Park
                .lineToLinearHeading(new Pose2d(park, Math.toRadians(90)))
                //.turn(right90)
                .waitSeconds(.2)

                .build();
    }

    @Override
    protected Pose2d getStartingPose(){
        return startPose;
    }

}
