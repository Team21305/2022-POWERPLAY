package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Left: 3-Cone", group="Blue")
public class LeftBlue extends ThreeConeJunctionOnly{

    public LeftBlue() { super(true, 30.5, new Vector2d(), new Vector2d());  }
}