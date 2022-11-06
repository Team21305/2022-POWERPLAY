package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@TeleOp
public class TeleOpMode extends CommandOpMode {
    private GamepadEx gamepad;

    private DriveSubsystem driveSubsystem;
    private LiftSubsystem liftSubsystem;
    private IntakeSubSystem intakeSubsystem;
    private Button downButton;
    private Button upButton;
    private Button openButton;
    private Button closeButton;
    private Button middleButton;
    private Button groundButton;
    private Button lowButton;

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry = multipleTelemetry;

        Hardware hardware = new Hardware(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardware, telemetry);
        liftSubsystem = new LiftSubsystem(hardware, multipleTelemetry);
        intakeSubsystem = new IntakeSubSystem(hardware, telemetry);

        gamepad = new GamepadEx(gamepad1);

        downButton = (new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(new InstantCommand(()->liftSubsystem.goToBottom()));

        upButton = (new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP))
                .whenPressed(new InstantCommand(()->liftSubsystem.goToTop()));

        middleButton = (new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT))
                .whenPressed(new InstantCommand(()->liftSubsystem.goToMiddle()));

        groundButton = (new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT))
                .whenPressed(new InstantCommand(()->liftSubsystem.goToGround()));

        lowButton = (new GamepadButton(gamepad, GamepadKeys.Button.X))
                .whenPressed(new InstantCommand(()->liftSubsystem.goToLow()));

        openButton = (new GamepadButton(gamepad, GamepadKeys.Button.Y))
                .whenPressed(new InstantCommand(()->intakeSubsystem.open()));

        closeButton = (new GamepadButton(gamepad, GamepadKeys.Button.A))
                .whenPressed(new InstantCommand(()->intakeSubsystem.close()));

        register(driveSubsystem, liftSubsystem, intakeSubsystem);
        driveSubsystem.setDefaultCommand(new DefaultDrive(
                driveSubsystem,
                () -> gamepad.getLeftY(),
                () -> gamepad.getLeftX(),
                () -> gamepad.getRightX()));
    }
}


