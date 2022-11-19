package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveForward;
import org.firstinspires.ftc.teamcode.commands.DriveStrafe;

import java.util.HashMap;

@Autonomous(name="Robot: Auto", group="Robot")
@Config
public class AutoOpMode extends RobotBaseOpMode {

    public static double ForwardDistance=30;
    public static double StrafeDistance=28;
    public static double AutoSpeed=0.7;

    @Override
    public void initialize(){
        super.initialize();

        Command left = new DriveStrafe(driveSubsystem, StrafeDistance, -AutoSpeed)
                .andThen(new DriveForward(driveSubsystem, ForwardDistance, AutoSpeed));
        Command center = new DriveForward(driveSubsystem, ForwardDistance, AutoSpeed);
        Command right = new DriveStrafe(driveSubsystem, StrafeDistance, AutoSpeed)
                .andThen(new DriveForward(driveSubsystem, ForwardDistance, AutoSpeed));

        auto.whenPressed(new DriveStrafe(driveSubsystem, StrafeDistance, -AutoSpeed)
                .andThen(new DriveForward(driveSubsystem, ForwardDistance, AutoSpeed)));
        auto2.whenPressed(new DriveForward(driveSubsystem, ForwardDistance, AutoSpeed));
        auto3.whenPressed(new DriveStrafe(driveSubsystem, StrafeDistance, AutoSpeed)
                .andThen(new DriveForward(driveSubsystem, ForwardDistance, AutoSpeed)));

        SequentialCommandGroup leftSeq = new SequentialCommandGroup(
                new InstantCommand(()->intakeSubsystem.close()),
                new WaitCommand(1000),
                left,
                new RunCommand(()->driveSubsystem.stop())
        );

        SequentialCommandGroup centerSeq = new SequentialCommandGroup(
                new InstantCommand(()->intakeSubsystem.close()),
                new WaitCommand(1000),
                center,
                new RunCommand(()->driveSubsystem.stop())
        );

        SequentialCommandGroup rightSeq = new SequentialCommandGroup(
                new InstantCommand(()->intakeSubsystem.close()),
                new WaitCommand(1000),
                right,
                new RunCommand(()->driveSubsystem.stop())
        );

        SelectCommand autoCommand = new SelectCommand(
                // the first parameter is a map of commands
                new HashMap<Object, Command>() {{
                    put(Position.LEFT, leftSeq);
                    put(Position.CENTER, centerSeq);
                    put(Position.RIGHT, rightSeq);
                }},
                // the selector
                this::getPosition
        );

        schedule(new WaitCommand(5000).andThen(autoCommand));

        telemetry.addLine("autoOpInit conplete");
        telemetry.update();
    }

    public enum Position {
        LEFT, CENTER, RIGHT
    }

    public Position getPosition() {
        // some code to detect height of the starter stack

        int tagID = visionSubSystem.getTag();

        if(tagID == 0){
            return Position.LEFT;
        }

        if(tagID == 1){
            return Position.CENTER;
        }

        if(tagID == 2){
            return Position.RIGHT;
        }

        return Position.CENTER;
    }



}
