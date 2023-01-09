package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.opmode.core.Adjustments;

@TeleOp(name="RR", group="TEST")
public class RightRed extends ThreeConeTerminalBase2{

    RightRed() {
        super(false, 39.5);
    }
}
