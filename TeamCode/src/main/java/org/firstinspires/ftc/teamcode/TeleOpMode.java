package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
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
import org.firstinspires.ftc.teamcode.commands.DriveForward;
import org.firstinspires.ftc.teamcode.commands.DriveStrafe;
import org.firstinspires.ftc.teamcode.commands.PurePursuit;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@TeleOp
@Config
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

    public static double ForwardDistance=30;
    public static double StrafeDistance=28;
    public static double AutoSpeed=0.7;

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

        DriveForward dfCommand = new DriveForward(driveSubsystem, ForwardDistance, AutoSpeed);
        DriveStrafe dsCommand = new DriveStrafe(driveSubsystem, StrafeDistance,AutoSpeed);

        Button auto = (new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP))
                .whenPressed(new DriveForward(driveSubsystem, ForwardDistance, AutoSpeed));

        Button auto2 = (new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(new DriveStrafe(driveSubsystem, StrafeDistance,AutoSpeed));

        Button auto3 = (new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT))
                .whenPressed(new ScheduleCommand(dsCommand.andThen(dfCommand)));

        register(driveSubsystem, liftSubsystem, intakeSubsystem);
        driveSubsystem.setDefaultCommand(new DefaultDrive(
                driveSubsystem,
                () -> gamepad.getLeftY(),
                () -> gamepad.getLeftX(),
                () -> gamepad.getRightX()));


        telemetry.addLine("Init Complete");
        telemetry.update();
    }
}


