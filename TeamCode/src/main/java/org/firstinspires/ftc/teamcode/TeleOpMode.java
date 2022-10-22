package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpMode extends OpMode {
    private Hardware hardware;
    private MecanumDrive mecanumDrive;
    private GamepadEx gamepad;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);

        mecanumDrive = new MecanumDrive(
            hardware.driveLeftFront,
            hardware.driveRightFront,
            hardware.driveLeftRear,
            hardware.driveRightRear
        );

        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        mecanumDrive.driveRobotCentric(
            -gamepad.getLeftX(),
            -gamepad.getLeftY(),
            -gamepad.getRightX(),
                true
        );

        if(gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            hardware.liftServo0.setPosition(0);
            hardware.liftServo2.setPosition(0);
        }

        if(gamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
            hardware.liftServo0.setPosition(1);
            hardware.liftServo2.setPosition(1);
        }
    }
}
