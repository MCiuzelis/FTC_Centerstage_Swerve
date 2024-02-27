package org.firstinspires.ftc.teamcode.hardware;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.hardware.Globals.planeLockPosition;

import androidx.annotation.GuardedBy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Config
public class RobotHardware{
    public static double nominalBatteryVoltage = 12.5;

    Telemetry telemetry;
    HardwareMap hw;

    public DcMotorEx FLT_Motor, FLB_Motor, FRT_Motor, FRB_Motor, BT_Motor, BB_Motor, SlideMotor;

    public DigitalChannel frontRightCalibrationSensor, frontLeftCalibrationSensor, backCalibrationSensor;
    List<LynxModule> allHubs;
    PhotonLynxVoltageSensor voltageSensor;

    public Servo clawAngleServo, clawLeftServo, clawRightServo, planeServo, axonLeft, axonRight;
    public AnalogInput armAxonEncoder;

    TrackBallDriver trackBall;


    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
//    public RevIMU imu;
    public RevIMU imu;
    public Rotation2d imuAngle = new Rotation2d();

    public DistanceSensor distanceSensor;

    Thread imuThread;


    public RobotHardware(HardwareMap hw){
        this.hw = hw;
    }




    public void initialiseHardware(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        allHubs = hw.getAll(LynxModule.class);
        voltageSensor = hw.getAll(PhotonLynxVoltageSensor.class).iterator().next();

        SlideMotor = initMotor(hw, "CH_Motor_1", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLT_Motor = initMotor(hw, "CH_Motor_2", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLB_Motor = initMotor(hw, "CH_Motor_3", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRT_Motor = initMotor(hw, "EH_Motor_1", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRB_Motor = initMotor(hw, "EH_Motor_0", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        BT_Motor = initMotor(hw, "EH_Motor_2", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        BB_Motor = initMotor(hw, "CH_Motor_0", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);

        clawAngleServo = initServo(hw, "EH_Servo_3", Servo.Direction.REVERSE);
        clawLeftServo = initServo(hw, "EH_Servo_1", Servo.Direction.FORWARD);
        clawRightServo = initServo(hw, "EH_Servo_2", Servo.Direction.REVERSE);
        axonLeft = initServo(hw, "EH_Servo_5", Servo.Direction.FORWARD);
        axonRight = initServo(hw, "EH_Servo_4", Servo.Direction.FORWARD);
        planeServo = initServo(hw, "EH_Servo_0", Servo.Direction.REVERSE);

        planeServo.setPosition(planeLockPosition);

        frontLeftCalibrationSensor = hw.get(DigitalChannel.class,"CH_Digital_7");
        frontRightCalibrationSensor = hw.get(DigitalChannel.class,"EH_Digital_1");
        backCalibrationSensor = hw.get(DigitalChannel.class,"CH_Digital_6");

        trackBall = hw.get(TrackBallDriver.class, "trackball");

        //armAxonEncoder = hw.get(AnalogInput.class, "EH_analog");


        synchronized (imuLock) {
            imu = new RevIMU(hw, "imu");
            imu.init();
            imu.reset();
        }


        distanceSensor = hw.get(DistanceSensor.class, "distance_sensor");
    }





    public void setBulkCachingMode(LynxModule.BulkCachingMode mode){
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(mode);
        }
    }

    public void clearBulkCache(){
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }

    private DcMotorEx initMotor(HardwareMap hw, String motorPort, DcMotorEx.Direction direction, DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        DcMotorEx motor = hw.get(DcMotorEx.class, motorPort);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    private Servo initServo (HardwareMap hw, String servoPort, Servo.Direction direction){
        Servo servo = hw.get(Servo.class, servoPort);
        servo.setDirection(direction);
        return servo;
    }

    public void startIMUThread(LinearOpMode opMode){
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                synchronized (imuLock) {
                    imuAngle = imu.getRotation2d();
                }
            }
        });
        imuThread.start();
    }

    public double getVoltageDriveMultiplier(){
        return clamp(Math.pow(voltageSensor.getCachedVoltage() / nominalBatteryVoltage, 2), 0.4, 1);
    }


    public void updateIMUStupidMonkeyMethod(){
        imuAngle = imu.getRotation2d();
    }

    public void getCalibrationSensorTelemetry(){
        telemetry.addData("calibrationSensorFL", frontLeftCalibrationSensor.getState());
        telemetry.addData("calibrationSensorFR", frontRightCalibrationSensor.getState());
        telemetry.addData("calibrationSensorB", backCalibrationSensor.getState());
    }
}