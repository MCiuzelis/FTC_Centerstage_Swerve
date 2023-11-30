package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
public class DrivetrainSubsystem extends SubsystemBase {

    public RobotHardware robot;
    public Telemetry telemetry;
    public boolean DEBUG_MODE;

    Rotation2d frontLeftModuleAngle = Rotation2d.fromDegrees(0);
    Rotation2d frontRightModuleAngle = Rotation2d.fromDegrees(0);
    Rotation2d backModuleAngle = Rotation2d.fromDegrees(0);

    double targetRobotAngle = 0;
    double prevFrontLeftDriveVelocity = 0;
    double prevFrontRightDriveVelocity = 0;
    double prevBackDriveVelocity = 0;

    ElapsedTime angleHoldingTimer;

    SwerveModule FrontLeft, FrontRight, Back;
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLocation);

    public DrivetrainSubsystem(RobotHardware hw, Telemetry telemetry, boolean isMode_DEBUG){
        robot = hw;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DEBUG_MODE = isMode_DEBUG;

        FrontLeft = new SwerveModule(robot.FLT_Motor, robot.FLB_Motor, robot.limitFL, -137, telemetry);
        FrontRight = new SwerveModule(robot.FRT_Motor, robot.FRB_Motor, robot.limitFR, 141, telemetry);
        Back = new SwerveModule(robot.BT_Motor, robot.BB_Motor, robot.limitB, 115, telemetry);
        angleHoldingTimer = new ElapsedTime();
    }

    public void ResetAllEncoders(){
        FrontLeft.resetModuleEncoders();
        FrontRight.resetModuleEncoders();
        Back.resetModuleEncoders();
    }

    public void Drive(Vector2d joystickDrive, double joystickTurnSpeed) {

        double driveSpeed = joystickDrive.magnitude();
        double driveAngle = joystickDrive.angle();

        Rotation2d imuAngle = robot.imu.getRotation2d();
        double turnPower;

        if (joystickTurnSpeed == 0) {
            if (angleHoldingTimer.milliseconds() > angleHoldingDelayMs) {
                double error = targetRobotAngle - imuAngle.getRadians();
                if (Math.abs(error) > Math.toRadians(180)) {
                    error += (-Math.signum(error) * Math.toRadians(360));
                }
                if (Math.abs(error) < allowedRobotAngleError){
                    turnPower = 0;
                }
                else{
                    turnPower = robotAngleHoldingKp * error;
                }
                telemetry.addLine("maintainingPosition");
            }
            else {
                turnPower = 0;
                targetRobotAngle = imuAngle.getRadians();
            }
        }
        else{
            targetRobotAngle = imuAngle.getRadians();
            turnPower = joystickTurnSpeed * K_rotation;
            angleHoldingTimer.reset();
            telemetry.addLine("not maintaining");
        }

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                driveSpeed * Math.sin(driveAngle) * highestPossibleMotorVelocity,
                driveSpeed * Math.cos(driveAngle) * highestPossibleMotorVelocity,
                turnPower,
                imuAngle
        );

        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState back = moduleStates[2];

        if (frontLeft.speedMetersPerSecond != 0){
            frontLeftModuleAngle = frontLeft.angle;
        }
        if (frontRight.speedMetersPerSecond != 0){
            frontRightModuleAngle = frontRight.angle;
        }
        if (back.speedMetersPerSecond != 0){
            backModuleAngle = back.angle;
        }

        FrontLeft.updateTurningAndDrivingSpeeds(frontLeftModuleAngle, frontLeft.speedMetersPerSecond);
        FrontRight.updateTurningAndDrivingSpeeds(frontRightModuleAngle, frontRight.speedMetersPerSecond);
        Back.updateTurningAndDrivingSpeeds(backModuleAngle, back.speedMetersPerSecond);

        double frontLeftModuleAvailableVelocity = highestPossibleMotorVelocity - Math.abs(FrontLeft.turnVelocityTicks);
        double frontRightModuleAvailableVelocity = highestPossibleMotorVelocity - Math.abs(FrontRight.turnVelocityTicks);
        double backModuleAvailableVelocity = highestPossibleMotorVelocity - Math.abs(Back.turnVelocityTicks);

        double frontLeftModuleAvailableAndRequestedVelocityRatio = Math.abs(frontLeftModuleAvailableVelocity / FrontLeft.driveSpeedMultiplier);
        double frontRightModuleAvailableAndRequestedVelocityRatio = Math.abs(frontRightModuleAvailableVelocity / FrontRight.driveSpeedMultiplier);
        double backModuleAvailableAndRequestedVelocityRatio = Math.abs(backModuleAvailableVelocity / Back.driveSpeedMultiplier);

        double smallestAvailableAndRequestedVelocityRatio = Math.min(Math.min(frontLeftModuleAvailableAndRequestedVelocityRatio, frontRightModuleAvailableAndRequestedVelocityRatio), backModuleAvailableAndRequestedVelocityRatio);

        double frontLeftModuleFinalDriveVelocity;
        double frontRightModuleFinalDriveVelocity;
        double backModuleFinalDriveVelocity;

        if (smallestAvailableAndRequestedVelocityRatio >= 1) {
            frontLeftModuleFinalDriveVelocity = FrontLeft.driveSpeedMultiplier;
            frontRightModuleFinalDriveVelocity = FrontRight.driveSpeedMultiplier;
            backModuleFinalDriveVelocity = Back.driveSpeedMultiplier;
        } else {
            frontLeftModuleFinalDriveVelocity = FrontLeft.driveSpeedMultiplier * smallestAvailableAndRequestedVelocityRatio;
            frontRightModuleFinalDriveVelocity = FrontRight.driveSpeedMultiplier * smallestAvailableAndRequestedVelocityRatio;
            backModuleFinalDriveVelocity = Back.driveSpeedMultiplier * smallestAvailableAndRequestedVelocityRatio;
        }

//        robot.FLT_Motor.setVelocity((FrontLeft.turnVelocityTicks) * finalAllMotorVelocityMultiplier);
//        robot.FLB_Motor.setVelocity((FrontLeft.turnVelocityTicks) * finalAllMotorVelocityMultiplier);
//        robot.FRT_Motor.setVelocity((FrontRight.turnVelocityTicks) * finalAllMotorVelocityMultiplier);
//        robot.FRB_Motor.setVelocity((FrontRight.turnVelocityTicks) * finalAllMotorVelocityMultiplier);
//        robot.BT_Motor.setVelocity((Back.turnVelocityTicks) * finalAllMotorVelocityMultiplier);
//        robot.BB_Motor.setVelocity((Back.turnVelocityTicks) * finalAllMotorVelocityMultiplier);

        frontLeftModuleFinalDriveVelocity = prevFrontLeftDriveVelocity * driveVelocitySmoothingRatio + (1 - driveVelocitySmoothingRatio) * frontLeftModuleFinalDriveVelocity;
        frontRightModuleFinalDriveVelocity = prevFrontRightDriveVelocity * driveVelocitySmoothingRatio + (1 - driveVelocitySmoothingRatio) * frontRightModuleFinalDriveVelocity;
        backModuleFinalDriveVelocity = prevBackDriveVelocity * driveVelocitySmoothingRatio + (1 - driveVelocitySmoothingRatio) * backModuleFinalDriveVelocity;

        prevFrontLeftDriveVelocity = frontLeftModuleFinalDriveVelocity;
        prevFrontRightDriveVelocity = frontRightModuleFinalDriveVelocity;
        prevBackDriveVelocity = backModuleFinalDriveVelocity;

        robot.FLT_Motor.setVelocity((FrontLeft.turnVelocityTicks + frontLeftModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.FLB_Motor.setVelocity((FrontLeft.turnVelocityTicks - frontLeftModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.FRT_Motor.setVelocity((FrontRight.turnVelocityTicks + frontRightModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.FRB_Motor.setVelocity((FrontRight.turnVelocityTicks - frontRightModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.BT_Motor.setVelocity((Back.turnVelocityTicks + backModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.BB_Motor.setVelocity((Back.turnVelocityTicks - backModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
    }

    public void stop(){
        robot.FLT_Motor.setPower(0);
        robot.FLB_Motor.setPower(0);
        robot.FRT_Motor.setPower(0);
        robot.FRB_Motor.setPower(0);
        robot.BT_Motor.setPower(0);
        robot.BB_Motor.setPower(0);
    }

    public void Calibrate(){
        FrontLeft.start();
        FrontRight.start();
        Back.start();
    }
}