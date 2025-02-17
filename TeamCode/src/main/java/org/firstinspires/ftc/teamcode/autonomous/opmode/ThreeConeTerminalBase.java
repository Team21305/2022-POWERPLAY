package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.autonomous.opmode.core.AutoBase;
import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;

public class ThreeConeTerminalBase extends AutoBase {

    private double flipOverX;
    private Boolean isQuad2;
    Vector2d coneStackPre;
    Vector2d coneStack;
    Vector2d terminal;
    Vector2d lowJunction;
    Vector2d lowJunctionF;
    Vector2d mediumJunction;
    Vector2d mediumJunctionF;
    Vector2d readSignal;

    Vector2d park;

    double right90;
    double left90;
    Pose2d startPose;

    ThreeConeTerminalBase(Boolean isQuad2, double startingX, Vector2d lowOffset, Vector2d mediumOffset) {

        flipOverX = isQuad2 ? -1 : 1;
        this.isQuad2 = isQuad2;

        coneStackPre = new Vector2d(-56, -12 * flipOverX);
        coneStack = new Vector2d(-58.5, -12 * flipOverX);
        terminal = new Vector2d(-57, -60 * flipOverX);
        lowJunction = new Vector2d(-47, -12 * flipOverX);
        lowJunctionF = new Vector2d(-47, -15 * flipOverX).plus(lowOffset);
        mediumJunction = new Vector2d(-23.5+medJunXOffset, -12 * flipOverX);
        mediumJunctionF = new Vector2d(-23.5, -15 * flipOverX).plus(mediumOffset);

        readSignal = new Vector2d(startingX, -60* flipOverX);

        park = new Vector2d(-12, -12 * flipOverX);

        right90 = Math.toRadians(-90 * flipOverX);
        left90 = Math.toRadians(90 * flipOverX);
        startPose = new Pose2d(startingX, -64.5 * flipOverX, Math.toRadians(90 * flipOverX));
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
                .lineToLinearHeading(new Pose2d(lowJunctionF, Math.toRadians(-90 * flipOverX)))
                // Drop Cone
            .addTemporalMarker(() -> intake.open())
                .lineToLinearHeading(new Pose2d(lowJunction, Math.toRadians(-90 * flipOverX)))



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
                .lineToLinearHeading(new Pose2d(mediumJunctionF, Math.toRadians(-90 * flipOverX)))

                // Drop Cone
            .addTemporalMarker(() -> intake.open())
                .lineToLinearHeading(new Pose2d(mediumJunction, Math.toRadians(-90 * flipOverX)))

                .waitSeconds(.2)


            .build();
    }

    @Override
    protected TrajectorySequence getParkingTrajectory(Pose2d beginPose) {

        int tag = vision.getTag();

        multipleTelemetry.addData("Parking using tag", tag);
        multipleTelemetry.update();

        double parkOffset = isQuad2 ? 0 : -44;

        if(tag == 1){
            parkOffset = -22;
        }
        if(tag == 2){
            parkOffset = isQuad2 ? -44: 0;
        }

        return drive
                .trajectorySequenceBuilder(beginPose)

                // Park
                .turn(right90)
                .lineToLinearHeading(new Pose2d(park.getX() + parkOffset, park.getY() ,Math.toRadians(180)))
                .turn(right90)
                .waitSeconds(.2)

                .build();
    }

    @Override
    protected Pose2d getStartingPose(){
        return startPose;
    }

}
