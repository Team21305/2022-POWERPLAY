package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.autonomous.opmode.core.AutoBase;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;

public class ThreeConeJunctionOnly extends AutoBase {

    private Boolean isQuad1;
    private double flipOverX;
    Vector2d coneStack;
    Vector2d preConeStack;
    Vector2d buryCone;
    Vector2d lowJunction45;
    Vector2d mediumJunction;

    Vector2d park;

    double right90;
    double left90;
    Pose2d startPose;

    ThreeConeJunctionOnly(Boolean isQuad1, double startingX){

        this.isQuad1 = isQuad1;

        flipOverX = isQuad1 ? -1 : 1;
        preConeStack = new Vector2d(36, -7.5 * flipOverX);
        buryCone = new Vector2d(58, -15.5 * flipOverX);
        coneStack = new Vector2d(58.5, -9.5 * flipOverX);
        lowJunction45 = new Vector2d(41, -31.5 * flipOverX);
        mediumJunction = new Vector2d(22.5, -9.5 * flipOverX);

        park = new Vector2d(12, -12 * flipOverX);

        right90 = Math.toRadians(-90 * flipOverX);
        left90 = Math.toRadians(90 * flipOverX);
        startPose = new Pose2d(startingX, -64.5 * flipOverX, Math.toRadians(90 * flipOverX));
    }

    @Override
    protected TrajectorySequence getSequence() {

        multipleTelemetry.addLine("getSequence is called");
        multipleTelemetry.update();

        // Written from the Right-Red station.  (Quadrant II)

        TrajectorySequence poseTest2 = drive
                .trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->intake.close())
                .addTemporalMarker(.3, ()->lift.goToLow())
                .lineToLinearHeading(new Pose2d(lowJunction45, Math.toRadians(flipOverX * 45)))
                // drop cone
                .addTemporalMarker(()->intake.open())
                .lineToLinearHeading(new Pose2d(preConeStack, 0))

                // drive to cone stack, bury cone
                .addTemporalMarker(()->lift.goToPosition(0.18))
                .splineTo(buryCone, Math.toRadians(flipOverX * -40))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(preConeStack, 0), 0)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(coneStack, 0))

                // pickup top cone
                .addTemporalMarker(()->intake.close())
                .waitSeconds(0.4)

                // go to medium juntion
                .addTemporalMarker(()->lift.goToMiddle())
                .lineToLinearHeading(new Pose2d(mediumJunction, 0))
                .turn(right90)

                // drop cone
                .addTemporalMarker(()->intake.open())
                .turn(left90)

                // To Cone Stack
                .addTemporalMarker(()->lift.goToPosition(0.14))
                .lineToLinearHeading(new Pose2d(coneStack, 0))

                // pickup cone
                .addTemporalMarker(()->intake.close())
                .waitSeconds(0.4)
                .addTemporalMarker( ()->lift.goToLow())

                // go to medium junction
                .lineToLinearHeading(new Pose2d(mediumJunction, 0))
                .addTemporalMarker(()->lift.goToMiddle())
                .turn(right90)

                // drop cone
                .addTemporalMarker(()->intake.open())
                .waitSeconds(0.2)

                // turn and go back to stack
                .turn(left90)
                .addTemporalMarker(()->lift.goToBottom())

                .build();

        return poseTest2;
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
