package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Right: 3-Cone", group="Red")
public class RightRed extends ThreeConeJunctionOnly{

    public RightRed() {
        super(false, 39.5);
    }
}
