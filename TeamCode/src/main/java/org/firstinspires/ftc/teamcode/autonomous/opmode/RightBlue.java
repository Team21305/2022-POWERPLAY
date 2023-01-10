package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Blue Right: 3-Cone", group="Blue")
public class RightBlue extends ThreeConeTerminalBase {

    public RightBlue() {
        super(true, -40, new Vector2d(0, -1), new Vector2d(0, -1));
    }
}