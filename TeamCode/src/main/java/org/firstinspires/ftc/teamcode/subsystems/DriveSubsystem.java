package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
public class  DriveSubsystem extends SubsystemBase {

    private final MecanumDrive mecanumDrive;
    @androidx.annotation.NonNull
    private final Hardware hardware;
    private final Telemetry telemetry;
    private final BNO055IMU imu;

    private Orientation angle;

    public static double kp = 1.0;
    public static double ki = 0.0000;

    public static boolean squareInputs = true;
    public static boolean isFieldCentric = false;

    public static Motor.RunMode runMode=Motor.RunMode.VelocityControl;

    public DriveSubsystem(Hardware hardware, MultipleTelemetry telemetry){
        imu = hardware.imu;
        this.hardware = hardware;

        this.telemetry = telemetry;

        hardware.driveLeftFront.setRunMode(runMode);
        hardware.driveLeftRear.setRunMode(runMode);
        hardware.driveRightFront.setRunMode(runMode);
        hardware.driveRightRear.setRunMode(runMode);


        mecanumDrive = new MecanumDrive(
        false,
                hardware.driveLeftFront,
                hardware.driveRightFront,
                hardware.driveLeftRear,
                hardware.driveRightRear
        );

        mecanumDrive.setMaxSpeed(1.0);

    }

    public void drive(double y, double x, double r){

        hardware.driveLeftFront.setFeedforwardCoefficients(0, 0,0);
        hardware.driveRightFront.setFeedforwardCoefficients(0, 0, 0);
        hardware.driveLeftRear.setFeedforwardCoefficients(0, 0, 0);
        hardware.driveRightRear.setFeedforwardCoefficients(0, 0, 0);

        hardware.driveLeftFront.setVeloCoefficients(kp, ki,0);
        hardware.driveRightFront.setVeloCoefficients(kp, ki, 0);
        hardware.driveLeftRear.setVeloCoefficients(kp, ki, 0);
        hardware.driveRightRear.setVeloCoefficients(kp, ki, 0);

        if (isFieldCentric){
            mecanumDrive.driveFieldCentric(x, y, r, angle.firstAngle, true);
        }else {
            mecanumDrive.driveRobotCentric(x, y, r, squareInputs);
        }

        telemetry.addData("y",y);
        telemetry.addData("x",x);
        telemetry.addData("r",r);
    }


    public void readEncoders(){


        double frontLeft = hardware.driveLeftFront.encoder.getCorrectedVelocity();
        double frontRight = hardware.driveRightFront.encoder.getCorrectedVelocity();
        double rearLeft = hardware.driveLeftRear.encoder.getCorrectedVelocity();
        double rearRight = hardware.driveRightRear.encoder.getCorrectedVelocity();

        telemetry.addData(">LeftFront", hardware.driveLeftFront.get());
        telemetry.addData(">RightFront", hardware.driveRightFront.get());
        telemetry.addData(">LeftRear", hardware.driveLeftRear.get());
        telemetry.addData(">RightRear", hardware.driveRightRear.get());

        telemetry.addData("fle", frontLeft);
        telemetry.addData("fre", frontRight);
        telemetry.addData("rle", rearLeft);
        telemetry.addData("rre", rearRight);



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
