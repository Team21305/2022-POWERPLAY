package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware;

import java.util.concurrent.TimeUnit;

@Config
public class  DriveSubsystem extends SubsystemBase {

    public final MecanumDrive mecanumDrive;
    @androidx.annotation.NonNull
    private final Hardware hardware;
    private final Telemetry telemetry;
    private final BNO055IMU imu;
    private final MecanumDriveOdometry odometry;

    private double angle;

    public static double kp = 1.0;
    public static double ki = 0.0006;

    public static boolean squareInputs = true;
    public static boolean isFieldCentric = true;

    private static final double WHEEL_DIAMETER_INCHES = 3.77952756;
    private static final double PULSES_PER_REV = 537.7;
    private static final double INCHES_PER_REV = WHEEL_DIAMETER_INCHES * Math.PI;
    private static final double DISTANCE_PER_PULSE = INCHES_PER_REV / PULSES_PER_REV;
    private static final double L1 = 4.33;
    private static final double L2 = 5.9;


    public static Motor.RunMode runMode=Motor.RunMode.VelocityControl;
    private Pose2d m_pose;

    public DriveSubsystem(Hardware hardware, MultipleTelemetry telemetry){
        imu = hardware.imu;
        this.hardware = hardware;
        this.telemetry = telemetry;

        hardware.driveLeftFront.setRunMode(runMode);
        hardware.driveLeftRear.setRunMode(runMode);
        hardware.driveRightFront.setRunMode(runMode);
        hardware.driveRightRear.setRunMode(runMode);

        hardware.driveLeftFront.setDistancePerPulse(DISTANCE_PER_PULSE);
        hardware.driveLeftRear.setDistancePerPulse(DISTANCE_PER_PULSE);
        hardware.driveRightFront.setDistancePerPulse(DISTANCE_PER_PULSE);
        hardware.driveRightRear.setDistancePerPulse(DISTANCE_PER_PULSE);

        // ***************************************************************
        // **** Odometry configurations ****
        // This code is copied/based on code found in this example:
        // https://docs.ftclib.org/ftclib/kinematics/wpilib-kinematics/mecanum-drive-odometry

        //          Front (x)
        //            ^
        //   \\ --------------- //
        //   \\ |             | //      ^
        //      |             |         | -- l1 (distance from center of robot to wheel center (x))
        //      |      .      |         v
        //      |             |
        //   // |             | \\
        //   // --------------- \\
        //
        //    <------->
        //    l2  (distance from center out to wheel center (y))

        // Locations of the wheels relative to the robot center.
        Translation2d m_frontLeftLocation =
                new Translation2d(L1, L2);
        Translation2d m_frontRightLocation =
                new Translation2d(L1, -L2);
        Translation2d m_backLeftLocation =
                new Translation2d(-L1, L2);
        Translation2d m_backRightLocation =
                new Translation2d(-L1, -L2);

        // Creating my kinematics object using the wheel locations.
        MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics
                (
                        m_frontLeftLocation, m_frontRightLocation,
                        m_backLeftLocation, m_backRightLocation
                );

        // Creating my odometry object from the kinematics object. Here,
        // our starting pose is 5 meters along the long end of the field and in the
        // center of the field along the short end, facing forward.
        odometry = new MecanumDriveOdometry
        (
            m_kinematics, getGyroHeading(),
            new Pose2d(0, 0, new Rotation2d())
        );

        mecanumDrive = new MecanumDrive(
        false,
                hardware.driveLeftFront,
                hardware.driveRightFront,
                hardware.driveLeftRear,
                hardware.driveRightRear
        );

        // ***************************************************************

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
            mecanumDrive.driveFieldCentric(x, y, r, angle, true);
        }else {
            mecanumDrive.driveRobotCentric(x, y, r, squareInputs);
        }

        telemetry.addData("y",y);
        telemetry.addData("x",x);
        telemetry.addData("r",r);
    }

    public void stop() {
        mecanumDrive.stop();
    }

    public void readEncoders(){

        double frontLeft = hardware.driveLeftFront.getRate();
        double frontRight = hardware.driveRightFront.getRate();
        double rearLeft = hardware.driveLeftRear.getRate();
        double rearRight = hardware.driveRightRear.getRate();

        telemetry.addData(">LeftFront", hardware.driveLeftFront.get());
        telemetry.addData(">RightFront", hardware.driveRightFront.get());
        telemetry.addData(">LeftRear", hardware.driveLeftRear.get());
        telemetry.addData(">RightRear", hardware.driveRightRear.get());

        telemetry.addData("fle", frontLeft);
        telemetry.addData("fre", frontRight);
        telemetry.addData("rle", rearLeft);
        telemetry.addData("rre", rearRight);

    }

    public Rotation2d getGyroHeading(){
        return Rotation2d.fromDegrees(getGyroHeadingDegrees());
    }

    public double getGyroHeadingDegrees(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void readGyro (){
        angle = getGyroHeadingDegrees();
        telemetry.addData( "gyro", angle);
    }

    public void updateOdometry(){
        // Get my wheel speeds; assume .getRate() has been
        // set up to return velocity of the encoder
        // in meters per second.
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds
                (
                        hardware.driveLeftFront.getRate(),
                        hardware.driveRightFront.getRate(),
                        hardware.driveLeftRear.getRate(),
                        hardware.driveRightRear.getRate()
                );

        // Get my gyro angle.
        Rotation2d gyroAngle = getGyroHeading();

        TimeUnit.NANOSECONDS.toSeconds(System.nanoTime());

        // Update the pose
        m_pose = odometry.updateWithTime((double) System.nanoTime() / 1E9, gyroAngle, wheelSpeeds);

        telemetry.addData("Pose", m_pose);
    }

    public Pose2d getPose(){
        return m_pose;
    }


    @Override
    public void periodic() {
        super.periodic();

        readEncoders();
        readGyro();
        updateOdometry();


        telemetry.update();
    }
}
