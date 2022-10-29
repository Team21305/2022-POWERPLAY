package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive mecanumDrive;
    private final Telemetry telemetry;

    public DriveSubsystem(Hardware hardware, Telemetry telemetry){

        mecanumDrive = new MecanumDrive(
                hardware.driveLeftFront,
                hardware.driveRightFront,
                hardware.driveLeftRear,
                hardware.driveRightRear
        );
        this.telemetry = telemetry;
    }

    public void drive(double y, double x, double r){
        mecanumDrive.driveRobotCentric(x, y, r,true);
    }

    @Override
    public void periodic() {
        super.periodic();

        telemetry.addLine();
    }
}
