package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.HashMap;
import java.util.Map;

public class RobotHardware {

    Telemetry telemetry;
    //Motors and servos

    public Servo intakeHingeServo, depositorPinServo, depositorLeftServo, depositorRightServo, planeServo;

    public DcMotorEx depositorSlideMotor, intakeBroomMotor, FLT_Motor, FLB_Motor, FRT_Motor, FRB_Motor, BT_Motor, BB_Motor;

    public RevIMU imu;

    public TouchSensor limitFR, limitFL, limitB, limitIntake;

    private static RobotHardware instance = null;

    public static RobotHardware getInstance(){
        if (instance == null){
            instance = new RobotHardware();
        }
        return instance;
    }

    public void init(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;

        //init() connected devices
        //region Motors

        intakeBroomMotor = initMotor(hw, "eh2", DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.FLOAT);
        depositorSlideMotor = initMotor(hw, "ch2", DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLT_Motor = initMotor(hw, "ch1", DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLB_Motor = initMotor(hw, "ch0", DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRT_Motor = initMotor(hw, "eh0", DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRB_Motor = initMotor(hw, "eh1", DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE);
        BT_Motor = initMotor(hw, "eh3", DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE);
        BB_Motor = initMotor(hw, "ch3", DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE);

        //endregion
        //region Servos

        //depositorLeftServo = initServo(hw,"a",false,0,1,AngleUnit.DEGREES);
        //depositorRightServo = initServo(hw,"EHservo1",false,0,1,AngleUnit.DEGREES);
        //depositorPinServo = initServo(hw,"EHservo0",false,0,1,AngleUnit.DEGREES);
        //intakeHingeServo = initServo(hw,"CHservo3",false,0,1,AngleUnit.DEGREES);
//
        depositorLeftServo = initServo(hw,"EHservo2", Servo.Direction.FORWARD);
        depositorRightServo = initServo(hw,"EHservo1", Servo.Direction.FORWARD);
        depositorPinServo = initServo(hw,"EHservo0", Servo.Direction.FORWARD);
        intakeHingeServo = initServo(hw,"CHservo3", Servo.Direction.FORWARD);
        planeServo = initServo(hw, "EHservo5", Servo.Direction.FORWARD);


        //endregion
        //region Digital

        limitIntake = hw.get(TouchSensor.class, "CHdigital2");
        limitFL = hw.get(TouchSensor.class,"CHdigital2");
        limitFR = hw.get(TouchSensor.class,"EHdigital0");
        limitB = hw.get(TouchSensor.class,"EHdigital2");

        //endregion

        imu = new RevIMU(hw, "imu");
        imu.init();
    }

    private DcMotorEx initMotor(HardwareMap hw, String motorPort, DcMotorEx.Direction direction, DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        DcMotorEx motor = hw.get(DcMotorEx.class, motorPort);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }

    private ServoEx initServo(HardwareMap hw, String servoPort, boolean isInverted, double min, double max, AngleUnit angleUnit) {
        ServoEx servo = hw.get(ServoEx.class, servoPort);
        servo.setInverted(isInverted);
        servo.setRange(min, max, angleUnit);
        return servo;
    }

    private Servo initServo (HardwareMap hw, String servoPort, Servo.Direction direction){
        Servo servo = hw.get(Servo.class, servoPort);
        servo.setDirection(direction);
        return servo;
    }


}
