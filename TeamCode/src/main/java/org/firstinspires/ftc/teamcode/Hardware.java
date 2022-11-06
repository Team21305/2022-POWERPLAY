package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    public MotorEx driveLeftFront, driveRightFront, driveLeftRear, driveRightRear;
    public Servo liftServo0, liftServo2;
    public Servo liftServo4, liftServo5;
    public BNO055IMU imu;

    public Hardware(HardwareMap hardwareMap) {
        driveLeftFront = new MotorEx(hardwareMap, "driveLeftFront", Motor.GoBILDA.RPM_312);
        driveRightFront = new MotorEx(hardwareMap, "driveRightFront", Motor.GoBILDA.RPM_312);
        driveLeftRear = new MotorEx(hardwareMap, "driveLeftRear", Motor.GoBILDA.RPM_312);
        driveRightRear = new MotorEx(hardwareMap, "driveRightRear", Motor.GoBILDA.RPM_312);

        driveLeftFront.setInverted(true);
        driveLeftRear.setInverted(true);
        driveRightFront.setInverted(true);
        driveRightRear.setInverted(true);
        //driveRightFront.encoder.setDirection(Motor.Direction.REVERSE);
        //driveRightRear.encoder.setDirection(Motor.Direction.REVERSE);


        liftServo0 = hardwareMap.get(Servo.class, "servo0");
        liftServo2 = hardwareMap.get(Servo.class, "servo2");

        liftServo2.setDirection(Servo.Direction.REVERSE);

        liftServo4 = hardwareMap.get(Servo.class, "servo4");
        liftServo5 = hardwareMap.get(Servo.class, "servo5");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }
}
