package org.firstinspires.ftc.teamcode.hardware;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.hardware.Constants.nominalBatteryVoltage;
import static org.firstinspires.ftc.teamcode.hardware.Constants.planeLockPosition;

import androidx.annotation.GuardedBy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;

import java.util.List;

public class RobotHardware implements Runnable{

    Telemetry telemetry;
    HardwareMap hw;

    public DcMotorEx FLT_Motor, FLB_Motor, FRT_Motor, FRB_Motor, BT_Motor, BB_Motor, ArmMotor;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public RevIMU imu;
    Thread imuThread;


    ElapsedTime voltageTimer = new ElapsedTime();
    double voltageDriveMultiplier = 1;


    public DigitalChannel frontRightCalibrationSensor, frontLeftCalibrationSensor, backCalibrationSensor;
    List<LynxModule> allHubs;


    public Rotation2d imuAngle = Rotation2d.fromDegrees(0);
    public Servo clawAngleServo, clawLeftServo, clawRightServo, planeServo;



    public RobotHardware(HardwareMap hw){
        this.hw = hw;
    }




    public void initialiseHardware(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        allHubs = hw.getAll(LynxModule.class);

        ArmMotor = initMotor(hw, "ch0", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLT_Motor = initMotor(hw, "ch2", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLB_Motor = initMotor(hw, "ch3", DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRT_Motor = initMotor(hw, "eh1", DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRB_Motor = initMotor(hw, "eh0", DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE);
        BT_Motor = initMotor(hw, "eh2", DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE);
        BB_Motor = initMotor(hw, "ch1", DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE);

        clawAngleServo = initServo(hw, "CHservo0", Servo.Direction.FORWARD);

        clawLeftServo = initServo(hw, "CHservo1", Servo.Direction.FORWARD);
        clawRightServo = initServo(hw, "CHservo2", Servo.Direction.FORWARD);

        planeServo = initServo(hw, "EHservo0", Servo.Direction.REVERSE);
        planeServo.setPosition(planeLockPosition);


        frontLeftCalibrationSensor = hw.get(DigitalChannel.class,"CHdigital0");
        frontRightCalibrationSensor = hw.get(DigitalChannel.class,"EHdigital1");
        backCalibrationSensor = hw.get(DigitalChannel.class,"CHdigital1");

        imu = new RevIMU(hw, "imu");
        imu.init();

        //webcam = hw.get(WebcamName.class, "Webcam 1");
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
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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


    @Override
    public void run(){
        if (voltageTimer.seconds() > 4 || voltageDriveMultiplier == 0) {
            double voltage = hw.voltageSensor.iterator().next().getVoltage();
            voltageDriveMultiplier = clamp(voltage / nominalBatteryVoltage, 0.5, 1);
            voltageTimer.reset();
        }
    }



    public double getDriveMultiplierFromBatteryVoltage(){
        return voltageDriveMultiplier;
    }


    public void updateIMUStupidMonkeyMethod(){
        imuAngle = imu.getRotation2d();
    }


    public void getHardwareTelemetry(){
        telemetry.addData("calibrationSensorFL", frontLeftCalibrationSensor.getState());
        telemetry.addData("calibrationSensorFR", frontRightCalibrationSensor.getState());
        telemetry.addData("calibrationSensorB", backCalibrationSensor.getState());
    }
}