package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.opmode.core.Adjustments;

@Config
@Autonomous(name="Right Red: 3-Cone", group="3-Cone")
public class RightRed extends ThreeConeJunctionOnly{

    public static Adjustments myAdjustments = new Adjustments(1.0);

    RightRed() {
        super(false);
    }
}
