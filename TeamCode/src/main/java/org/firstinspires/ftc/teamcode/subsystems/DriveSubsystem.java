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
    private final MecanumDriveOdometry odometry;

    private double angle;

    public static double kp = 1.0;
    public static double ki = 0.0000;

    public static boolean squareInputs = true;
    public static boolean isFieldCentric = true;

    private static final double wheelDiameterInches = 0;
    private static final double pulsesPerRevolution = 0;
    private static final double inchesPerRevolution = 0;
    private static final double distancePerPulse = 1;
    private static final double wheelInchesFromCenter = 0.381;

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

        hardware.driveLeftFront.setDistancePerPulse(distancePerPulse);
        hardware.driveLeftRear.setDistancePerPulse(distancePerPulse);
        hardware.driveRightFront.setDistancePerPulse(distancePerPulse);
        hardware.driveRightRear.setDistancePerPulse(distancePerPulse);

        // ***************************************************************
        // **** Odometry configurations ****
        // This code is copied/based on code found in this example:
        // https://docs.ftclib.org/ftclib/kinematics/wpilib-kinematics/mecanum-drive-odometry

        // Locations of the wheels relative to the robot center.
        Translation2d m_frontLeftLocation =
                new Translation2d(0.381, 0.381);
        Translation2d m_frontRightLocation =
                new Translation2d(0.381, -0.381);
        Translation2d m_backLeftLocation =
                new Translation2d(-0.381, 0.381);
        Translation2d m_backRightLocation =
                new Translation2d(-0.381, -0.381);

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

        // Update the pose
        m_pose = odometry.updateWithTime((double) System.nanoTime() / 1E9, gyroAngle, wheelSpeeds);

        telemetry.addData("Pose", m_pose);
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
