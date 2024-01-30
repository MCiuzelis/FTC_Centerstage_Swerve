package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.Constants.planeLockPosition;
import androidx.annotation.GuardedBy;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.List;

public class RobotHardware{

    Telemetry telemetry;
    HardwareMap hw;

    public DcMotorEx FLT_Motor, FLB_Motor, FRT_Motor, FRB_Motor, BT_Motor, BB_Motor, SlideMotor;

    public DigitalChannel frontRightCalibrationSensor, frontLeftCalibrationSensor, backCalibrationSensor;
    List<LynxModule> allHubs;
    PhotonLynxVoltageSensor voltageSensor;

    public Servo clawAngleServo, clawLeftServo, clawRightServo, planeServo, axonLeft, axonRight;
    public AnalogInput armAxonEncoder;



    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public RevIMU imu;
    public Rotation2d imuAngle = new Rotation2d();

    private final Object distanceSensorLock = new Object();
    @GuardedBy("distanceSensorLock")
    public DistanceSensor distanceSensor;
    public double distance = 0;

    Thread imuThread, distanceSensorThread;


    public RobotHardware(HardwareMap hw){
        this.hw = hw;
    }




    public void initialiseHardware(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        allHubs = hw.getAll(LynxModule.class);
        voltageSensor = hw.getAll(PhotonLynxVoltageSensor.class).iterator().next();

        SlideMotor = initMotor(hw, "ch0", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLT_Motor = initMotor(hw, "ch2", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLB_Motor = initMotor(hw, "ch3", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRT_Motor = initMotor(hw, "eh1", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRB_Motor = initMotor(hw, "eh0", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        BT_Motor = initMotor(hw, "eh2", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);
        BB_Motor = initMotor(hw, "ch1", DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE);

        clawAngleServo = initServo(hw, "EHservo4", Servo.Direction.REVERSE);

        clawLeftServo = initServo(hw, "CHservo3", Servo.Direction.FORWARD);
        clawRightServo = initServo(hw, "CHservo4", Servo.Direction.REVERSE);

        axonLeft = initServo(hw, "EHservo3", Servo.Direction.FORWARD);
        axonRight = initServo(hw, "EHservo2", Servo.Direction.FORWARD);


        planeServo = initServo(hw, "EHservo0", Servo.Direction.REVERSE);
        planeServo.setPosition(planeLockPosition);

        frontLeftCalibrationSensor = hw.get(DigitalChannel.class,"CHdigital0");
        frontRightCalibrationSensor = hw.get(DigitalChannel.class,"EHdigital1");
        backCalibrationSensor = hw.get(DigitalChannel.class,"CHdigital1");

        armAxonEncoder = hw.get(AnalogInput.class, "axonArmEncoder");

        synchronized (imuLock) {
            imu = new RevIMU(hw, "imu");
            distanceSensor = hw.get(DistanceSensor.class, "distanceSensor");
            imu.init();
        }

        synchronized (distanceSensorLock){
            distanceSensor = hw.get(DistanceSensor.class, "distanceSensor");
        }
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
                    distance = distanceSensor.getDistance(DistanceUnit.CM);
                }
            }
        });
        imuThread.start();
    }

    public void startDistanceSensorThread(LinearOpMode opMode){
        distanceSensorThread = new Thread(() -> {
            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                synchronized (distanceSensorLock) {
                    distance = distanceSensor.getDistance(DistanceUnit.CM);
                }
            }
        });
        distanceSensorThread.start();
    }

    public double getVoltage(){
        return voltageSensor.getCachedVoltage();
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