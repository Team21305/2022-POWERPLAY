package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware;

public class  DriveSubsystem extends SubsystemBase {

    private final MecanumDrive mecanumDrive;
    private final Telemetry telemetry;
    private final Motor.Encoder frontleftencoder;
    private final Motor.Encoder frontrightencoder;
    private final Motor.Encoder rearleftencoder;
    private final Motor.Encoder rearrightencoder;
    private final BNO055IMU imu;

   private Orientation angle;

    public DriveSubsystem(Hardware hardware, Telemetry telemetry){
        imu = hardware.imu;

        this.telemetry = telemetry;

        frontleftencoder = hardware.driveLeftFront.encoder;
        frontrightencoder = hardware.driveRightFront.encoder;
        rearleftencoder = hardware.driveLeftRear.encoder;
        rearrightencoder = hardware.driveRightRear.encoder;

        hardware.driveLeftFront.setRunMode(Motor.RunMode.VelocityControl);
        hardware.driveRightFront.setRunMode(Motor.RunMode.VelocityControl);
        hardware.driveLeftRear.setRunMode(Motor.RunMode.VelocityControl);
        hardware.driveRightRear.setRunMode(Motor.RunMode.VelocityControl);

        double kp=1.2;
        hardware.driveLeftFront.setVeloCoefficients(kp, 0,0);
        hardware.driveRightFront.setVeloCoefficients(kp, 0, 0);
        hardware.driveLeftRear.setVeloCoefficients(kp, 0, 0);
        hardware.driveRightRear.setVeloCoefficients(kp, 0, 0);

        mecanumDrive = new MecanumDrive(
                hardware.driveLeftFront,
                hardware.driveRightFront,
                hardware.driveLeftRear,
                hardware.driveRightRear
        );

        mecanumDrive.setMaxSpeed(1.0);

    }

    public void drive(double y, double x, double r){
       // mecanumDrive.driveRobotCentric(x, y, r,true);
        mecanumDrive.driveFieldCentric(x, y, r, angle.firstAngle, true);
    }


    public void readEncoders(){

        double fle = frontleftencoder.getRevolutions();
        double fre = frontrightencoder.getRevolutions();
        double rle = rearleftencoder.getRevolutions();
        double rre = rearrightencoder.getRevolutions();

        telemetry.addData("fle", fle);
        telemetry.addData("fre", fre);
        telemetry.addData("rle", rle);
        telemetry.addData("rre", rre);



    }

    public void readGyro (){
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData( "gyro", angle.firstAngle);
    }


    @Override
    public void periodic() {
        super.periodic();

        readEncoders();
        readGyro();


        telemetry.update();
    }
}
