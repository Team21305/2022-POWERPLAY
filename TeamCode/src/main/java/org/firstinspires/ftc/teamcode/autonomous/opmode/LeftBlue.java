package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name="Left Blue -- 3-Cone", group="3-Cone")
public class LeftBlue extends ThreeConeJunctionOnly{

    LeftBlue() {
        super(true, -30.5);
    }
}