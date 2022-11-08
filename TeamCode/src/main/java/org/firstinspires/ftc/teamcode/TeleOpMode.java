package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.PurePursuit;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@TeleOp
public class TeleOpMode extends CommandOpMode {
    private GamepadEx gamepad;
    private GamepadEx gamepadCo;

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
        driveSubsystem = new DriveSubsystem(hardware, multipleTelemetry);
        liftSubsystem = new LiftSubsystem(hardware, multipleTelemetry);
        intakeSubsystem = new IntakeSubSystem(hardware, multipleTelemetry);

        gamepad = new GamepadEx(gamepad1);
        gamepadCo = new GamepadEx(gamepad2);

        downButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(new InstantCommand(()->liftSubsystem.goToBottom()));

        upButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.DPAD_UP))
                .whenPressed(new InstantCommand(()->liftSubsystem.goToTop()));

        middleButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.DPAD_LEFT))
                .whenPressed(new InstantCommand(()->liftSubsystem.goToMiddle()));

        groundButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.DPAD_RIGHT))
                .whenPressed(new InstantCommand(()->liftSubsystem.goToGround()));

        lowButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.X))
                .whenPressed(new InstantCommand(()->liftSubsystem.goToLow()));

        openButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.Y))
                .whenPressed(new InstantCommand(()->intakeSubsystem.open())
                        .andThen(new InstantCommand(()->liftSubsystem.goToBottom()))
                );

        closeButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.A))
                .whenPressed(new InstantCommand(()->intakeSubsystem.close()));

        // create our pure pursuit command
        PurePursuit ppCommand = new PurePursuit(
                driveSubsystem.mecanumDrive, driveSubsystem,
                new StartWaypoint(0, 0),
                //new GeneralWaypoint(15, 0, 0.5, 0.0, 6),
                //new GeneralWaypoint(16, 0, 1.0, 0.0, 2),
                //new GeneralWaypoint(18, 0, 0.5, 0.0, 6),
                new EndWaypoint(
                        36, 0, 0, 0.5,
                        0.0, 0, 12, 10
                )
        );


        Button auto = (new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP))
                .whenPressed(new InstantCommand(()->schedule(ppCommand)));


        register(driveSubsystem, liftSubsystem, intakeSubsystem);
        driveSubsystem.setDefaultCommand(new DefaultDrive(
                driveSubsystem,
                () -> gamepad.getLeftY(),
                () -> gamepad.getLeftX(),
                () -> gamepad.getRightX()));
    }
}


