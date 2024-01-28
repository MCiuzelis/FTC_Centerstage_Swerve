package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
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
import static org.firstinspires.ftc.teamcode.hardware.Constants.nominalBatteryVoltage;
import static org.firstinspires.ftc.teamcode.hardware.Constants.robotAngleAllowedErrorRads;
import static org.firstinspires.ftc.teamcode.hardware.Constants.robotAngleHoldingKp;
import static org.firstinspires.ftc.teamcode.hardware.Constants.turnVelocityMultiplierAtMaxSpeed;
import static org.firstinspires.ftc.teamcode.hardware.Constants.turnVelocityMultiplierCutoffValue;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
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

public class DrivetrainSubsystem extends SubsystemBase {

    RobotHardware hardware;
    Telemetry telemetry;
    boolean DEBUG_MODE;

    Rotation2d frontLeftModuleAngle = new Rotation2d();
    Rotation2d frontRightModuleAngle = new Rotation2d();
    Rotation2d backModuleAngle = new Rotation2d();

    double targetRobotAngleRads = 0;

    public SwerveModule FrontLeft, FrontRight, Back;
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLocation);
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d());

    Pose2d driveInput = new Pose2d();

    ElapsedTime odometryTimer = new ElapsedTime();
    Rotation2d imuOffset;

    public Pose2d robotPosition = new Pose2d();

    PIDEx turnPID = new PIDEx(new PIDCoefficientsEx(DriveBaseTurnKp, DriveBaseTurnKi, DriveBaseTurnKd,
                        DriveBaseTurnMaxIntegralSum, DriveBaseTurnStabilityThreshold, DriveBaseTurnLowPassGain));

    PIDEx drivePID = new PIDEx(new PIDCoefficientsEx(DriveBaseDriveKp, DriveBaseDriveKi, DriveBaseDriveKd,
                         DriveBaseDriveMaxIntegralSum, DriveBaseDriveStabilityThreshold, DriveBaseDriveLowPassGain));

    BasicFeedforward driveFeedForward = new BasicFeedforward(new FeedforwardCoefficients(DriveBaseDriveKv, 0, DriveBaseDriveKs));


    DRIVETRAIN_STATE currentState = DRIVETRAIN_STATE.FULLY_MANUAL;
    PIDController backboardPositionController = new PIDController(0, 0, 0);
    public double targetDistance = 100;





    public DrivetrainSubsystem(RobotHardware robot, Telemetry telemetry, double[] moduleAngleOffsetsRads, double imuOffsetRads, boolean isMode_DEBUG){
        this.hardware = robot;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DEBUG_MODE = isMode_DEBUG;

        FrontLeft = new SwerveModule(robot.FLT_Motor, robot.FLB_Motor, robot.frontLeftCalibrationSensor, -137, moduleAngleOffsetsRads[0], DEBUG_MODE, telemetry);
        FrontRight = new SwerveModule(robot.FRT_Motor, robot.FRB_Motor, robot.frontRightCalibrationSensor, 141, moduleAngleOffsetsRads[1], DEBUG_MODE, telemetry);
        Back = new SwerveModule(robot.BT_Motor, robot.BB_Motor, robot.backCalibrationSensor, 115, moduleAngleOffsetsRads[2], DEBUG_MODE, telemetry);

        this.imuOffset = new Rotation2d(imuOffsetRads);
        CommandScheduler.getInstance().registerSubsystem(this);
    }


    public DrivetrainSubsystem(RobotHardware robot, Telemetry telemetry, boolean DEBUG_MODE){
        this(robot, telemetry, new double[]{0, 0, 0}, 0, DEBUG_MODE);
    }




     public void loop(){

         SwerveModuleState[] moduleStates = translateChassisSpeedToModuleStates(
                 driveInput,
                 hardware.imuAngle.plus(imuOffset));

         SwerveModuleState frontLeft = moduleStates[0];
         SwerveModuleState frontRight = moduleStates[1];
         SwerveModuleState back = moduleStates[2];


         if (frontLeft.speedMetersPerSecond != 0) frontLeftModuleAngle = frontLeft.angle;
         if (frontRight.speedMetersPerSecond != 0) frontRightModuleAngle = frontRight.angle;
         if (back.speedMetersPerSecond != 0) backModuleAngle = back.angle;

         double turnVelocityMultiplier = getTurnVelocityMultiplier(getChassisSpeedFromEncoders());

         //old begin
//         double frontLeftTurnCorrection = FrontLeft.getTurnCorrection(frontLeftModuleAngle.getRadians(), turnPID) * turnVelocityMultiplier;
//         double frontRightTurnCorrection = FrontRight.getTurnCorrection(frontRightModuleAngle.getRadians(), turnPID) * turnVelocityMultiplier;
//         double backTurnCorrection = Back.getTurnCorrection(backModuleAngle.getRadians(), turnPID) * turnVelocityMultiplier;
//
//         double frontLeftDriveCorrection = FrontLeft.getDriveCorrection(frontLeft.speedMetersPerSecond, driveFeedForward, drivePID);
//         double frontRightDriveCorrection = FrontRight.getDriveCorrection(frontRight.speedMetersPerSecond, driveFeedForward, drivePID);
//         double backDriveCorrection = Back.getDriveCorrection(back.speedMetersPerSecond, driveFeedForward, drivePID);
//
//         double frontLeftAvailableAndRequestedPowerRatio = (1d - Math.abs(frontLeftTurnCorrection)) / Math.abs(frontLeftDriveCorrection);
//         double frontRightAvailableAndRequestedPowerRatio = (1d - Math.abs(frontRightTurnCorrection)) / Math.abs(frontRightDriveCorrection);
//         double backAvailableAndRequestedPowerRatio = (1d - Math.abs(backTurnCorrection)) / Math.abs(backDriveCorrection);
//
//         double smallestAvailableAndRequestedPowerRatio = Math.min(Math.min(frontLeftAvailableAndRequestedPowerRatio, frontRightAvailableAndRequestedPowerRatio), backAvailableAndRequestedPowerRatio);
//
//         if (smallestAvailableAndRequestedPowerRatio <= 1) {
//             frontLeftDriveCorrection *= smallestAvailableAndRequestedPowerRatio;
//             frontRightDriveCorrection *= smallestAvailableAndRequestedPowerRatio;
//             backDriveCorrection *= smallestAvailableAndRequestedPowerRatio;
//         }
//
//
//         FrontLeft.setTurnAndDrivePower(frontLeftTurnCorrection, frontLeftDriveCorrection);
//         FrontRight.setTurnAndDrivePower(frontRightTurnCorrection, frontRightDriveCorrection);
//         Back.setTurnAndDrivePower(backTurnCorrection, backDriveCorrection);

//
//         if (DEBUG_MODE){
//             telemetry.addData("frontLeftTurnCorrection", frontLeftTurnCorrection);
//             telemetry.addData("frontLeftDriveCorrection", frontLeftDriveCorrection);
//         }
         //old end


        //vectorMethod:
        Vector2d[] motorPowers = getScaledModuleVectors(
                FrontLeft.getModuleMotorVelocities(frontLeft.speedMetersPerSecond, driveFeedForward, drivePID, frontLeftModuleAngle.getRadians(), turnPID, turnVelocityMultiplier),
                FrontRight.getModuleMotorVelocities(frontRight.speedMetersPerSecond, driveFeedForward, drivePID, frontRightModuleAngle.getRadians(), turnPID, turnVelocityMultiplier),
                Back.getModuleMotorVelocities(back.speedMetersPerSecond, driveFeedForward, drivePID, backModuleAngle.getRadians(), turnPID, turnVelocityMultiplier));

        FrontLeft.setPowers(motorPowers[0].getX(), motorPowers[0].getY());
        FrontRight.setPowers(motorPowers[1].getX(), motorPowers[1].getY());
        Back.setPowers(motorPowers[2].getX(), motorPowers[2].getY());
     }


     public void setModeToMaintainDistance(double targetRobotAngleRads, double targetDistance){
        currentState = DRIVETRAIN_STATE.MAINTAIN_DISTANCE;
        this.targetRobotAngleRads = targetRobotAngleRads;
        this.targetDistance = targetDistance;
     }


    public void setModeToManual(){
        currentState = DRIVETRAIN_STATE.FULLY_MANUAL;
        targetRobotAngleRads = hardware.imuAngle.getRadians();
    }


     public enum DRIVETRAIN_STATE{
         FULLY_MANUAL,
         MAINTAIN_DISTANCE
     }


    public void drive(Pose2d gamepadInput){
        switch (currentState){
            case FULLY_MANUAL:
                driveInput = gamepadInput;
                break;

            case MAINTAIN_DISTANCE:
                double correctionX = backboardPositionController.calculate(hardware.distance, targetDistance);
                driveInput = new Pose2d(correctionX, driveInput.getY(), new Rotation2d(0));
                break;
        }
    }

    public void drive(){
        drive(new Pose2d());
    }


    public Vector2d[] getScaledModuleVectors(Vector2d... moduleVectors){
        double maxPower = 1;
        for (Vector2d moduleVector : moduleVectors) {
            double currentMax = Math.max(Math.abs(moduleVector.getX()), Math.abs(moduleVector.getY()));
            if (currentMax > maxPower) maxPower = currentMax;
        }

        Vector2d[] output = new Vector2d[moduleVectors.length];
        for (int i = 0; i < moduleVectors.length; i++) output[i] = moduleVectors[i].div(maxPower);
        return output;
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

        if (currentAndMaxVelocityRatio < turnVelocityMultiplierCutoffValue) {
            return 1;
        } else {
            return 1d + currentAndMaxVelocityRatio * (turnVelocityMultiplierAtMaxSpeed - 1d);
        }
    }


    public SwerveModuleState[] translateChassisSpeedToModuleStates(Pose2d driveInput, Rotation2d imuAngle){
        double voltageMultiplier = clamp(Math.pow(hardware.getVoltage() / nominalBatteryVoltage, 2), 0.5, 1);
        return kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        driveInput.getY() * highestPossibleMotorVelocity * voltageMultiplier,
                        -driveInput.getX() * highestPossibleMotorVelocity * voltageMultiplier,
                        getTurnPower(driveInput.getHeading(), imuAngle.getRadians()) * voltageMultiplier,
                        imuAngle
                ));
    }


    public double getTurnPower(double turnSpeed, double imuAngle){
        double angleError = targetRobotAngleRads - imuAngle;

        if (turnSpeed == 0 && Math.abs(angleError) > robotAngleAllowedErrorRads) {
            return  robotAngleHoldingKp * angleError;
        }
        else{
            targetRobotAngleRads = imuAngle;
            return turnSpeed * K_rotation;
        }
    }


    public void updateOdometryFromMotorEncoders(){
        SwerveModuleState frontLeftState =
                new SwerveModuleState(FrontLeft.getDrivingVelocity(), FrontLeft.getAngleRotation2d());
        SwerveModuleState frontRightState =
                new SwerveModuleState(FrontRight.getDrivingVelocity(), FrontRight.getAngleRotation2d());
        SwerveModuleState backState =
                new SwerveModuleState(Back.getDrivingVelocity(), Back.getAngleRotation2d());
        robotPosition = odometry.updateWithTime(odometryTimer.seconds(), hardware.imuAngle, frontLeftState, frontRightState, backState);
    }


    public void resetAllEncoders(){
        stopAllMotors();
        FrontLeft.resetModuleEncoders();
        FrontRight.resetModuleEncoders();
        Back.resetModuleEncoders();
    }


    public void stopAllMotors(){
        drive();
        FrontLeft.stopMotors();
        FrontRight.stopMotors();
        Back.stopMotors();
    }


    public void calibrate(){
        Thread calibrateFrontLeft = new Thread(FrontLeft);
        Thread calibrateFrontRight = new Thread(FrontRight);
        Thread calibrateBack = new Thread(Back);

        calibrateFrontLeft.start();
        calibrateFrontRight.start();
        calibrateBack.start();
    }


    public double[] getAllModuleAngleRads(){
        return new double[]{FrontLeft.getNormalisedAngleRads(), FrontRight.getNormalisedAngleRads(), Back.getNormalisedAngleRads()};
    }


    public void resetImuOffset(){
        imuOffset = hardware.imuAngle;
    }
}