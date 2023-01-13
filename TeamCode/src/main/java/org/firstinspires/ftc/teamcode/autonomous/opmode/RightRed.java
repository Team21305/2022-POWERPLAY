package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Right: 3-Cone", group="Red")
public class RightRed extends ThreeConeJunctionOnly{

    public RightRed() {
        super(false, 39.5, new Vector2d(), new Vector2d());
    }
}
