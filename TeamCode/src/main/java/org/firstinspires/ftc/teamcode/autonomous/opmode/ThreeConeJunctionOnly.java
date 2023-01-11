package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.autonomous.opmode.core.AutoBase;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;

public class ThreeConeJunctionOnly extends AutoBase {

    private final double flipOverX;
    private final Vector2d coneStack;
    private final Vector2d preConeStack;
    //private final Vector2d buryCone;
    private final Vector2d lowJunction45;
    private final Vector2d mediumJunction;
    private final Vector2d mediumJunctionF;

    private final Vector2d park;

    private final double right90;
    private final double left90;
    private final Pose2d startPose;
    private final Boolean isQuad1;
    private final Vector2d readSignal;
    private final Vector2d terminal;

    ThreeConeJunctionOnly(Boolean isQuad1, double startingX){

        flipOverX = isQuad1 ? -1 : 1;
        this.isQuad1 = isQuad1;

        readSignal = new Vector2d(startingX, -60* flipOverX);
        terminal = new Vector2d(57, -60 * flipOverX);

        preConeStack = new Vector2d(56, -12 * flipOverX);
        //buryCone = new Vector2d(58, -15.5 * flipOverX);
        coneStack = new Vector2d(58.5, -12 * flipOverX);
        lowJunction45 = new Vector2d(57, -26 * flipOverX);
        mediumJunction = new Vector2d(22.5, -9.5 * flipOverX);
        mediumJunctionF = new Vector2d(23.5, -12 * flipOverX);

        park = new Vector2d(56, -12 * flipOverX);

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

                // read signal
                .lineToLinearHeading(new Pose2d(readSignal, Math.toRadians(90 * flipOverX)))
                .waitSeconds(.1)

                // To Terminal
                .turn(right90)
                .lineToLinearHeading(new Pose2d(terminal, Math.toRadians(0)))

                // To Low Junction
                .turn(left90)
                .addTemporalMarker(()->lift.goToLow())
                .lineToLinearHeading(new Pose2d(lowJunction45, Math.toRadians(flipOverX * 90)))
                .turn(left90)
                //.lineToLinearHeading(new Pose2d(lowJunction45, Math.toRadians(flipOverX * 180)))

                // drop cone
                .addTemporalMarker(()->intake.open())
                .turn(right90)
                .lineToLinearHeading(new Pose2d(preConeStack, Math.toRadians(90 * flipOverX)))
                .turn(right90)

                // drive to cone stack, bury cone
                .addTemporalMarker(()->lift.goToPosition(0.18))
                //.splineTo(buryCone, Math.toRadians(flipOverX * -40))
                //.setReversed(true)
                //.splineToLinearHeading(new Pose2d(preConeStack, 0), 0)
                //.setReversed(false)
                .lineToLinearHeading(new Pose2d(coneStack, 0))

                // pickup top cone
                .addTemporalMarker(()->intake.close())
                .waitSeconds(0.4)

                // go to medium juntion
                .addTemporalMarker(()->lift.goToMiddle())
                .lineToLinearHeading(new Pose2d(mediumJunction, 0))
                .turn(right90)
                .lineToLinearHeading(new Pose2d(mediumJunctionF, Math.toRadians(-90 * flipOverX)))

                // drop cone
                .addTemporalMarker(()->intake.open())
                .lineToLinearHeading(new Pose2d(mediumJunction, Math.toRadians(-90 * flipOverX)))
                .turn(left90)

                // To Cone Stack
                .addTemporalMarker(()->lift.goToPosition(0.14))
                .lineToLinearHeading(new Pose2d(coneStack, 0))

                // pickup cone
                .addTemporalMarker(()->intake.close())
                .waitSeconds(0.4)
                .addTemporalMarker( ()->lift.goToLow())

//                // go to medium junction
//                .lineToLinearHeading(new Pose2d(mediumJunction, 0))
//                .addTemporalMarker(()->lift.goToMiddle())
//                .turn(right90)
//
//                // drop cone
//                .addTemporalMarker(()->intake.open())
//                .waitSeconds(0.2)
//
//                // turn and go back to stack
//                .turn(left90)
//                .addTemporalMarker(()->lift.goToBottom())

                .build();

        return poseTest2;
    }

    @Override
    protected TrajectorySequence getParkingTrajectory(Pose2d beginPose) {

        int tag = vision.getTag();

        multipleTelemetry.addData("Parking using tag", tag);
        multipleTelemetry.update();

        double parkOffset = isQuad1 ? 0 : -44;
        double yOffset = isQuad1 ? 5 : -5;

        if(tag == 1){
            parkOffset = -22;
        }
        if(tag == 2){
            parkOffset = isQuad1 ? -44: 0;
        }

        return drive
                .trajectorySequenceBuilder(beginPose)

                // Park
                .lineToLinearHeading(new Pose2d(park.getX() + parkOffset, park.getY() ,Math.toRadians(0)))
                .turn(left90)
                .lineToLinearHeading(new Pose2d(park.getX() + parkOffset, park.getY() + yOffset, Math.toRadians(90 * flipOverX)))
                .addTemporalMarker(()->lift.goToBottom())
                .waitSeconds(.5)


                .build();
    }

    @Override
    protected Pose2d getStartingPose(){
        return startPose;
    }

}
