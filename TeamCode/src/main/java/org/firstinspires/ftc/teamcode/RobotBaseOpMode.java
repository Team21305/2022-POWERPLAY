package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubSystem;

public class RobotBaseOpMode extends CommandOpMode {

    public GamepadEx gamepad;
    public GamepadEx gamepadCo;

    public DriveSubsystem driveSubsystem;
    public LiftSubsystem liftSubsystem;
    public IntakeSubSystem intakeSubsystem;
    //public VisionSubSystem visionSubSystem;

    public Button downButton;
    public Button upButton;
    public Button openButton;
    public Button closeButton;
    public Button middleButton;
    public Button groundButton;
    public Button lowButton;
    public Button makyItGoUpy;
    public Button makyItGoDowny;
    public Button justOpenClaw;
    public Button auto;
    public Button auto2;
    public Button auto3;
    public Button reset;
    public Button sideStackUp;
    public Button sideStackDown;
    public Button driveToWall;


    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Hardware hardware = new Hardware(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardware, multipleTelemetry);
        liftSubsystem = new LiftSubsystem(hardware, multipleTelemetry);
        intakeSubsystem = new IntakeSubSystem(hardware, multipleTelemetry);
        //visionSubSystem = new VisionSubSystem(hardware, multipleTelemetry);

        gamepad = new GamepadEx(gamepad1);
        gamepadCo = new GamepadEx(gamepad2);

        sideStackUp = (new GamepadButton(gamepadCo, GamepadKeys.Button.START));
        sideStackDown = (new GamepadButton(gamepadCo, GamepadKeys.Button.BACK));
        closeButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.A));
        openButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.Y));
        lowButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.X));
        groundButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.DPAD_RIGHT));
        middleButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.DPAD_LEFT));
        upButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.DPAD_UP));
        downButton = (new GamepadButton(gamepadCo, GamepadKeys.Button.DPAD_DOWN));
        makyItGoUpy = (new GamepadButton(gamepadCo, GamepadKeys.Button.RIGHT_BUMPER));
        makyItGoDowny = (new GamepadButton(gamepadCo, GamepadKeys.Button.LEFT_BUMPER));
        justOpenClaw = (new GamepadButton(gamepadCo, GamepadKeys.Button.B));
        reset = (new GamepadButton(gamepad, GamepadKeys.Button.Y));

        auto = (new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT));
        auto2 = (new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP));
        auto3 = (new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT));
        driveToWall = (new GamepadButton(gamepad, GamepadKeys.Button.X));

        register(driveSubsystem, liftSubsystem, intakeSubsystem);






    }
}
