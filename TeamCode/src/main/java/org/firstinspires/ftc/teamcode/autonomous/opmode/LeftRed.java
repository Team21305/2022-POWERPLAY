package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.trajectorysequence.TrajectorySequence;

@Autonomous(name="Left Red: 3-Cone", group="3-Cone")
public class LeftRed extends ThreeConeTerminalBase {

    public LeftRed() {
        super(false);
    }
}
