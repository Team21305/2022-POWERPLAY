package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    public MotorEx driveLeftFront, driveRightFront, driveLeftRear, driveRightRear;
    public Servo liftServo0;

    public Hardware(HardwareMap hardwareMap) {
        driveLeftFront = new MotorEx(hardwareMap, "driveLeftFront", Motor.GoBILDA.RPM_312);
        driveRightFront = new MotorEx(hardwareMap, "driveRightFront", Motor.GoBILDA.RPM_312);
        driveLeftRear = new MotorEx(hardwareMap, "driveLeftRear", Motor.GoBILDA.RPM_312);
        driveRightRear = new MotorEx(hardwareMap, "driveRightRear", Motor.GoBILDA.RPM_312);

        liftServo0 = hardwareMap.get(Servo.class, "servo0");

    }
}
