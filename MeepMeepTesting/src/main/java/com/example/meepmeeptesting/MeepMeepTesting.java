package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double multiplier = 1.0;
        double xOffset = -60;
        double yOffset = 24;
        Pose2d startPose = new Pose2d(-30, -60, Math.toRadians(180));

        Pose2d translate = new Pose2d(1, -1, -1);

        // Vectors
        Vector2d coneStack = new Vector2d(-56, -12);
        Vector2d terminal = new Vector2d(-56, -60);
        Vector2d lowTerminal = new Vector2d(-48, -12);
        Vector2d mediumTerminal = new Vector2d(-24, -12);

        Vector2d park = new Vector2d(-12, -12);

        double right90 = Math.toRadians(-90);
        double left90 = Math.toRadians(90);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setStartPose(startPose)
                .setDimensions(13, 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)

                                .lineToLinearHeading(new Pose2d(terminal, Math.toRadians(180)))
                                .turn(right90)

                                // To Cone Stack
                                .lineToLinearHeading(new Pose2d(coneStack, Math.toRadians(90)))
                                .turn(left90)

                                // To Low Junction
                                .lineToLinearHeading(new Pose2d(lowTerminal, Math.toRadians(180)))
                                .turn(left90)

                                // To Cone Stack
                                .turn(right90)
                                .lineToLinearHeading(new Pose2d(coneStack, Math.toRadians(180)))

                                // To Medium Junction
                                .lineToLinearHeading(new Pose2d(mediumTerminal, Math.toRadians(180)))
                                .turn(left90)

                                // Park
                                .lineToLinearHeading(new Pose2d(park, Math.toRadians(90)))
                                //.turn(right90)
                                .waitSeconds(2)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}