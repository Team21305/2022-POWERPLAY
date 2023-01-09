package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Left Red: 3-Cone", group="3-Cone")
public class LeftRed extends ThreeConeTerminalBase {

    public LeftRed() {
        super(false, -30.5, new Vector2d(2, 2), new Vector2d(2, 2));
    }
}
