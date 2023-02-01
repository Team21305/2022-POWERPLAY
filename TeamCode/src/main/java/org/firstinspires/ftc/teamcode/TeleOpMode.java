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
import org.firstinspires.ftc.teamcode.commands.DriveToWall;
import org.firstinspires.ftc.teamcode.commands.PurePursuit;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@TeleOp
@Config
public class TeleOpMode extends RobotBaseOpMode {

    public static double DISTANCE_TO_WALL = 6.0;
    public static double DISTANCE_TO_WALL_SPEED = 0.6;

    @Override
    public void initialize() {
        super.initialize();

        downButton.whenPressed(new InstantCommand(()->liftSubsystem.goToBottom()));
        upButton.whenPressed(new InstantCommand(()->liftSubsystem.goToTop()));
        middleButton.whenPressed(new InstantCommand(()->liftSubsystem.goToMiddle()));
        groundButton.whenPressed(new InstantCommand(()->liftSubsystem.goToGround()));
        lowButton.whenPressed(new InstantCommand(()->liftSubsystem.goToLow()));
        makyItGoUpy.whenPressed(new InstantCommand(()->liftSubsystem.bumpUp()));
        makyItGoDowny.whenPressed(new InstantCommand(()->liftSubsystem.bumpDown()));
        sideStackUp.whenPressed(new InstantCommand(()->liftSubsystem.sideStackUp()));
        sideStackDown.whenPressed(new InstantCommand(()->liftSubsystem.sideStackDown()));
        driveToWall.whenPressed(new DriveToWall(driveSubsystem, DISTANCE_TO_WALL, DISTANCE_TO_WALL_SPEED));


        openButton.whenPressed(new InstantCommand(()->intakeSubsystem.open())
                        .andThen(new InstantCommand(()->liftSubsystem.goToBottom()))
                );

        closeButton.whenPressed(new InstantCommand(()->intakeSubsystem.close()));

        justOpenClaw.whenPressed(new InstantCommand(()->intakeSubsystem.open()));

        reset.whenPressed(new InstantCommand(()-> driveSubsystem.resetGyro()));

        auto2.whenPressed(new DriveStrafe(driveSubsystem, 60, 0.7));
        auto.whenPressed(new DriveStrafe(driveSubsystem, 60, -0.7));


        driveSubsystem.setDefaultCommand(new DefaultDrive(
                driveSubsystem,
                liftSubsystem,
                ()-> gamepad.getLeftY(),
                () -> gamepad.getLeftX(),
                () -> gamepad.getRightX()));

        telemetry.clearAll();

        telemetry.addLine("Init Complete");

        telemetry.update();
    }
}


