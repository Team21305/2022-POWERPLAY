package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Left: 3-Cone", group="Blue")
public class LeftBlue extends ThreeConeJunctionOnly{

    public LeftBlue() { super(true, -30.5);  }
}