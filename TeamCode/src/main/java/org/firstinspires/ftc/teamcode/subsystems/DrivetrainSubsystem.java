package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
public class DrivetrainSubsystem extends SubsystemBase {
    public static double highestPossibleMotorVelocity = 2390;
    public static double turnVelocityMultiplierAtMaxSpeed = 0.75;
    public static double turnVelocityMultiplierCutoffValue = 0.8;

    public static double K_rotation = 5000;
    public static double angleHoldingKp = 10000;
    public static double turnDelay = 0.5;

    Translation2d frontLeftLocation = new Translation2d(0.0750555, 0.13);
    Translation2d frontRightLocation = new Translation2d(0.0750555, -0.13);
    Translation2d backLocation = new Translation2d(-0.15111, 0);


    boolean DEBUG_MODE;
    boolean isAutonomous;
    RobotHardware hardware;
    Telemetry telemetry;

    Rotation2d frontLeftModuleAngle = new Rotation2d();
    Rotation2d frontRightModuleAngle = new Rotation2d();
    Rotation2d backModuleAngle = new Rotation2d();

    double targetRobotAngleRads = 0;

    public SwerveModule FrontLeft, FrontRight, Back;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;

    Pose2d driveInput = new Pose2d();

    ElapsedTime odometryTimer = new ElapsedTime();
    double prevOdometryTime = 0;
    ElapsedTime delayTimer = new ElapsedTime();
    public com.arcrobotics.ftclib.geometry.Pose2d robotPosition = new com.arcrobotics.ftclib.geometry.Pose2d();
    Pose2d currentPos = new Pose2d();

    Rotation2d imuOffset;
    ChassisSpeeds chassisSpeeds;



    public DrivetrainSubsystem(RobotHardware robot, Telemetry telemetry, double imuOffsetRads, boolean isMode_DEBUG, boolean isAutonomous) {
        this.hardware = robot;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.isAutonomous = isAutonomous;
        DEBUG_MODE = isMode_DEBUG;

        FrontLeft = new SwerveModule(robot.FLT_Motor, robot.FLB_Motor, robot.frontLeftCalibrationSensor, DEBUG_MODE, telemetry);
        FrontRight = new SwerveModule(robot.FRT_Motor, robot.FRB_Motor, robot.frontRightCalibrationSensor, DEBUG_MODE, telemetry);
        Back = new SwerveModule(robot.BT_Motor, robot.BB_Motor, robot.backCalibrationSensor, DEBUG_MODE, telemetry);

        this.imuOffset = new Rotation2d(imuOffsetRads);
        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLocation);
        odometry = new SwerveDriveOdometry(kinematics, hardware.imu.getRotation2d());

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public DrivetrainSubsystem(RobotHardware robot, Telemetry telemetry, boolean DEBUG_MODE, boolean isAutonomous){
        this(robot, telemetry, 0, DEBUG_MODE, isAutonomous);
    }

     public void drive(){
         //ChassisSpeeds chassisSpeed = getChassisSpeedFromEncoders();
         //double turnVelocityMultiplier = getTurnVelocityMultiplier(chassisSpeed);
         double turnVelocityMultiplier = 1;

         SwerveModuleState[] moduleStates = translateChassisSpeedToModuleStates(
                 driveInput,
                 hardware.imuAngle.plus(imuOffset));

         SwerveModuleState frontLeft = moduleStates[0];
         SwerveModuleState frontRight = moduleStates[1];
         SwerveModuleState back = moduleStates[2];

         if (frontLeft.speedMetersPerSecond != 0) frontLeftModuleAngle = frontLeft.angle;
         if (frontRight.speedMetersPerSecond != 0) frontRightModuleAngle = frontRight.angle;
         if (back.speedMetersPerSecond != 0) backModuleAngle = back.angle;

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
                FrontLeft.getModuleMotorVelocities(frontLeft.speedMetersPerSecond, frontLeftModuleAngle.getRadians(), turnVelocityMultiplier),
                FrontRight.getModuleMotorVelocities(frontRight.speedMetersPerSecond, frontRightModuleAngle.getRadians(), turnVelocityMultiplier),
                Back.getModuleMotorVelocities(back.speedMetersPerSecond, backModuleAngle.getRadians(), turnVelocityMultiplier));

        FrontLeft.setPowers(motorPowers[0].getX(), motorPowers[0].getY());
        FrontRight.setPowers(motorPowers[1].getX(), motorPowers[1].getY());
        Back.setPowers(motorPowers[2].getY(), motorPowers[2].getX());
     }

    public void setGamepadInput(Pose2d gamepadInput){
        driveInput = gamepadInput;
    }

    public Pose2d getGamepadInput (){
        return driveInput;
    }

    public void setGamepadInput(){
        setGamepadInput(new Pose2d());
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

    public void updateChassisSpeedFromEncoders(){
        SwerveModuleState frontLeftState =
                new SwerveModuleState(FrontLeft.getDrivingVelocity(), FrontLeft.getAngleRotation2d());
        SwerveModuleState frontRightState =
                new SwerveModuleState(FrontRight.getDrivingVelocity(), FrontRight.getAngleRotation2d());
        SwerveModuleState backState =
                new SwerveModuleState(Back.getDrivingVelocity(), Back.getAngleRotation2d());
        chassisSpeeds = kinematics.toChassisSpeeds(frontLeftState, frontRightState, backState);
    }

    public ChassisSpeeds getChassisSpeedFromEncoders(){
        return chassisSpeeds;
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

    public void updateOdometryNew(){
        ChassisSpeeds speed = kinematics.toChassisSpeeds(FrontLeft.positionModuleState(),
                                                         FrontRight.positionModuleState(),
                                                         Back.positionModuleState());

        robotPosition = new com.arcrobotics.ftclib.geometry.Pose2d(robotPosition.getX() + speed.vyMetersPerSecond,
                                                                   robotPosition.getY() - speed.vxMetersPerSecond,
                                                                      hardware.imuAngle);
    }

    public com.acmerobotics.roadrunner.geometry.Vector2d getRobotXYVelocity(){
        return new com.acmerobotics.roadrunner.geometry.Vector2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    public Pose2d getRobotPosition(){
        double currentTime = odometryTimer.seconds();
        double dt = currentTime - prevOdometryTime;
        prevOdometryTime = currentTime;

        currentPos = new Pose2d(currentPos.getX() + chassisSpeeds.vxMetersPerSecond * dt,
                                currentPos.getY() + chassisSpeeds.vyMetersPerSecond * dt,
                                   hardware.imuAngle.getRadians());
        return currentPos;
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
        double voltageMultiplier = hardware.getVoltageDriveMultiplier();
        double vxPower = driveInput.getY() * highestPossibleMotorVelocity * voltageMultiplier;
        double vyPower = driveInput.getX() * highestPossibleMotorVelocity * voltageMultiplier;
        double turnPower = getTurnPower(driveInput.getHeading(), imuAngle.getRadians(), driveInput) * voltageMultiplier;

        if (isAutonomous) return kinematics.toSwerveModuleStates(new ChassisSpeeds(vxPower, vyPower, driveInput.getHeading() * K_rotation));
        else return kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(vxPower, vyPower, turnPower, imuAngle));
    }

    public double getTurnPower(double turnSpeed, double imuAngle, Pose2d driveInput){
        double angleError = targetRobotAngleRads - imuAngle;
        double driveMagnitude = Math.hypot(driveInput.getX(), driveInput.getY());

        if (turnSpeed == 0 && driveMagnitude != 0) {
            if (Math.abs(angleError) > Math.PI){
                angleError -= Math.signum(angleError) * 2d * Math.PI;
            }
            return angleHoldingKp * driveMagnitude * angleError;
        }

        else{
            if (turnSpeed != 0 || delayTimer.seconds() < turnDelay) targetRobotAngleRads = imuAngle;
            else delayTimer.reset();
            return turnSpeed * K_rotation;
        }
    }

    public void resetAllEncoders(){
        FrontLeft.resetModuleEncoders();
        FrontRight.resetModuleEncoders();
        Back.resetModuleEncoders();
    }

    public void stopAllMotors(){
        setGamepadInput();
        FrontLeft.stopMotors();
        FrontRight.stopMotors();
        Back.stopMotors();
    }

    public void calibrate() {
        Thread calibrateFrontLeft = new Thread(FrontLeft);
        Thread calibrateFrontRight = new Thread(FrontRight);
        Thread calibrateBack = new Thread(Back);

        calibrateFrontLeft.start();
        calibrateFrontRight.start();
        calibrateBack.start();
    }

    public boolean areModulesCalibrated(){
        return FrontLeft.moduleCalibrated && FrontRight.moduleCalibrated && Back.moduleCalibrated;
    }

    public double[] getAllModuleAngleRads(){
        return new double[]{FrontLeft.getNormalisedAngleRads(), FrontRight.getNormalisedAngleRads(), Back.getNormalisedAngleRads()};
    }

    public void resetImuOffset(){
        imuOffset = new Rotation2d();
        hardware.imu.reset();
    }

    public void updateModuleAngles(){
        FrontLeft.updateAngle();
        FrontRight.updateAngle();
        Back.updateAngle();
    }
}