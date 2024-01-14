package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseDriveKd;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseDriveKi;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseDriveKp;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseDriveKs;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseDriveKv;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseDriveLowPassGain;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseDriveMaxIntegralSum;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseDriveStabilityThreshold;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseTurnKd;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseTurnKi;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseTurnKp;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseTurnLowPassGain;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseTurnMaxIntegralSum;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseTurnStabilityThreshold;
import static org.firstinspires.ftc.teamcode.hardware.Constants.K_rotation;
import static org.firstinspires.ftc.teamcode.hardware.Constants.backLocation;
import static org.firstinspires.ftc.teamcode.hardware.Constants.frontLeftLocation;
import static org.firstinspires.ftc.teamcode.hardware.Constants.frontRightLocation;
import static org.firstinspires.ftc.teamcode.hardware.Constants.highestPossibleMotorVelocity;
import static org.firstinspires.ftc.teamcode.hardware.Constants.robotAngleHoldingKp;
import static org.firstinspires.ftc.teamcode.hardware.Constants.robotAngleHoldingKpStatic;
import static org.firstinspires.ftc.teamcode.hardware.Constants.turnVelocityMultiplierAtMaxSpeed;
import static org.firstinspires.ftc.teamcode.hardware.Constants.turnVelocityMultiplierCutoffValue;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
public class DrivetrainSubsystem extends SubsystemBase implements Runnable{

    RobotHardware robot;
    Telemetry telemetry;
    boolean DEBUG_MODE;

    public Rotation2d frontLeftModuleAngle = new Rotation2d();
    public Rotation2d frontRightModuleAngle = new Rotation2d();
    public Rotation2d backModuleAngle = new Rotation2d();

    double targetRobotAngle = 0;

    public SwerveModule FrontLeft, FrontRight, Back;
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLocation);

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(0));

    Vector2d joystickDrive = new Vector2d(0, 0);
    double joystickTurnSpeed = 0;

    public ElapsedTime odometryTimer = new ElapsedTime();
    Rotation2d imuOffset;

    public Pose2d robotPosition = new Pose2d();





    public DrivetrainSubsystem(RobotHardware robot, Telemetry telemetry, double[] moduleAngleOffsetsRads, double imuOffsetRads, boolean isMode_DEBUG){
        this.robot = robot;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DEBUG_MODE = isMode_DEBUG;

        FrontLeft = new SwerveModule(robot.FLT_Motor, robot.FLB_Motor, robot.frontLeftCalibrationSensor, -137, moduleAngleOffsetsRads[0], DEBUG_MODE, telemetry);
        FrontRight = new SwerveModule(robot.FRT_Motor, robot.FRB_Motor, robot.frontRightCalibrationSensor, 141, moduleAngleOffsetsRads[1], DEBUG_MODE, telemetry);
        Back = new SwerveModule(robot.BT_Motor, robot.BB_Motor, robot.backCalibrationSensor, 115, moduleAngleOffsetsRads[2], DEBUG_MODE, telemetry);

        this.imuOffset = new Rotation2d(imuOffsetRads);
    }


    public DrivetrainSubsystem(RobotHardware robot, Telemetry telemetry, boolean DEBUG_MODE){
        this.robot = robot;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.DEBUG_MODE = DEBUG_MODE;

        FrontLeft = new SwerveModule(robot.FLT_Motor, robot.FLB_Motor, robot.frontLeftCalibrationSensor, -137, 0, DEBUG_MODE, telemetry);
        FrontRight = new SwerveModule(robot.FRT_Motor, robot.FRB_Motor, robot.frontRightCalibrationSensor, 141, 0, DEBUG_MODE, telemetry);
        Back = new SwerveModule(robot.BT_Motor, robot.BB_Motor, robot.backCalibrationSensor, 115, 0, DEBUG_MODE, telemetry);
        imuOffset = Rotation2d.fromDegrees(0);
    }


    @Override
    public void run(){while (true) periodic();
    }




    public void periodic() {

        double driveSpeed = joystickDrive.magnitude();
        double driveAngle = joystickDrive.angle();
        Rotation2d imuAngle = robot.imuAngle.plus(imuOffset);

        SwerveModuleState[] moduleStates = translateChassisSpeedToModuleStates(driveSpeed, driveAngle, imuAngle);

        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState back = moduleStates[2];

        if (frontLeft.speedMetersPerSecond != 0) frontLeftModuleAngle = frontLeft.angle;
        if (frontRight.speedMetersPerSecond != 0) frontRightModuleAngle = frontRight.angle;
        if (back.speedMetersPerSecond != 0) backModuleAngle = back.angle;



        PIDEx turnPID = new PIDEx(new PIDCoefficientsEx(DriveBaseTurnKp, DriveBaseTurnKi, DriveBaseTurnKd,
                DriveBaseTurnMaxIntegralSum, DriveBaseTurnStabilityThreshold, DriveBaseTurnLowPassGain));

        PIDEx drivePID = new PIDEx(new PIDCoefficientsEx(DriveBaseDriveKp, DriveBaseDriveKi, DriveBaseDriveKd,
                DriveBaseDriveMaxIntegralSum, DriveBaseDriveStabilityThreshold, DriveBaseDriveLowPassGain));

        BasicFeedforward driveFeedForward = new BasicFeedforward(new FeedforwardCoefficients(DriveBaseDriveKv, 0, DriveBaseDriveKs));


        double turnVelocityMultiplier = getTurnVelocityMultiplier(getChassisSpeedFromEncoders());

        double frontLeftTurnCorrection = FrontLeft.getTurnCorrection(frontLeftModuleAngle.getRadians(), turnPID) * turnVelocityMultiplier;
        double frontRightTurnCorrection = FrontRight.getTurnCorrection(frontRightModuleAngle.getRadians(), turnPID) * turnVelocityMultiplier;
        double backTurnCorrection = Back.getTurnCorrection(backModuleAngle.getRadians(), turnPID) * turnVelocityMultiplier;

        double frontLeftDriveCorrection = FrontLeft.getDriveCorrection(frontLeft.speedMetersPerSecond, driveFeedForward, drivePID);
        double frontRightDriveCorrection = FrontRight.getDriveCorrection(frontRight.speedMetersPerSecond, driveFeedForward, drivePID);
        double backDriveCorrection = Back.getDriveCorrection(back.speedMetersPerSecond, driveFeedForward, drivePID);

        double frontLeftAvailableAndRequestedPowerRatio = (1d - Math.abs(frontLeftTurnCorrection)) / Math.abs(frontLeftDriveCorrection);
        double frontRightAvailableAndRequestedPowerRatio = (1d - Math.abs(frontRightTurnCorrection)) / Math.abs(frontRightDriveCorrection);
        double backAvailableAndRequestedPowerRatio = (1d - Math.abs(backTurnCorrection)) / Math.abs(backDriveCorrection);

        double smallestAvailableAndRequestedPowerRatio = Math.min(Math.min(frontLeftAvailableAndRequestedPowerRatio, frontRightAvailableAndRequestedPowerRatio), backAvailableAndRequestedPowerRatio);

        if (smallestAvailableAndRequestedPowerRatio <= 1) {
            frontLeftDriveCorrection *= smallestAvailableAndRequestedPowerRatio;
            frontRightDriveCorrection *= smallestAvailableAndRequestedPowerRatio;
            backDriveCorrection *= smallestAvailableAndRequestedPowerRatio;
        }

        if (DEBUG_MODE){
            telemetry.addData("frontLeftTurnCorrection", frontLeftTurnCorrection);
            telemetry.addData("frontLeftDriveCorrection", frontLeftDriveCorrection);
        }

        FrontLeft.setTurnAndDrivePower(frontLeftTurnCorrection, frontLeftDriveCorrection);
        FrontRight.setTurnAndDrivePower(frontRightTurnCorrection, frontRightDriveCorrection);
        Back.setTurnAndDrivePower(backTurnCorrection, backDriveCorrection);

        UpdateOdometry();
    }





    public void drive(Vector2d driveVector, double turnSpeed){
        joystickDrive = driveVector;
        joystickTurnSpeed = turnSpeed;
    }





    public ChassisSpeeds getChassisSpeedFromEncoders(){
        SwerveModuleState frontLeftState =
                new SwerveModuleState(FrontLeft.getDrivingVelocity(), FrontLeft.getAngleRotation2d());
        SwerveModuleState frontRightState =
                new SwerveModuleState(FrontRight.getDrivingVelocity(), FrontRight.getAngleRotation2d());
        SwerveModuleState backState =
                new SwerveModuleState(Back.getDrivingVelocity(), Back.getAngleRotation2d());

        return kinematics.toChassisSpeeds(frontLeftState, frontRightState, backState);
    }




    public double getTurnVelocityMultiplier(ChassisSpeeds chassisSpeeds) {
        double velocityVectorMagnitude = Math.hypot(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond);
        double currentAndMaxVelocityRatio = velocityVectorMagnitude / highestPossibleMotorVelocity;
        telemetry.addData("current and max velocity ratio", currentAndMaxVelocityRatio);
        if (currentAndMaxVelocityRatio < turnVelocityMultiplierCutoffValue) {
            return 1;
        } else {
            return 1d + (currentAndMaxVelocityRatio) * (turnVelocityMultiplierAtMaxSpeed - 1d);
        }
    }




    public SwerveModuleState[] translateChassisSpeedToModuleStates(double driveSpeed, double driveAngle, Rotation2d imuAngle){
        double turnPower = getTurnPower(driveSpeed, imuAngle.getRadians());
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                driveSpeed * Math.sin(driveAngle) * highestPossibleMotorVelocity,
                driveSpeed * -Math.cos(driveAngle) * highestPossibleMotorVelocity,
                turnPower,
                imuAngle
        );
        return kinematics.toSwerveModuleStates(speeds);
    }




    public double getTurnPower(double driveSpeed, double imuAngle){
        if (joystickTurnSpeed == 0) {
            double error = targetRobotAngle - imuAngle;
            return  (robotAngleHoldingKp * driveSpeed + robotAngleHoldingKpStatic) * error;
        }
        else{
            targetRobotAngle = imuAngle;
            return joystickTurnSpeed * K_rotation;
        }
    }




    public void UpdateOdometry(){
            SwerveModuleState frontLeftState =
                    new SwerveModuleState(FrontLeft.getDrivingVelocity(), FrontLeft.getAngleRotation2d());
            SwerveModuleState frontRightState =
                    new SwerveModuleState(FrontRight.getDrivingVelocity(), FrontRight.getAngleRotation2d());
            SwerveModuleState backState =
                    new SwerveModuleState(Back.getDrivingVelocity(), Back.getAngleRotation2d());
            robotPosition = odometry.updateWithTime(odometryTimer.seconds(), robot.imuAngle, frontLeftState, frontRightState, backState);
    }





    public void resetAllEncoders(){
        stopAllMotors();
        FrontLeft.resetModuleEncoders();
        FrontRight.resetModuleEncoders();
        Back.resetModuleEncoders();
    }




    public void stopAllMotors(){
        drive(new Vector2d(0, 0), 0);
        FrontLeft.stopMotors();
        FrontRight.stopMotors();
        Back.stopMotors();
    }




    public void calibrate(){
        FrontLeft.calibrate();
        FrontRight.calibrate();
        Back.calibrate();
    }



    public double[] getAllModuleAngleRads(){
        return new double[]{FrontLeft.forteleop(), FrontRight.forteleop(), Back.forteleop()};
    }
}