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

    SwerveModule FrontLeft, FrontRight, Back;
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLocation);

    Vector2d joystickDrive = new Vector2d(0, 0);
    double joystickTurnSpeed = 0;
    Rotation2d imuOffset;




    public DrivetrainSubsystem(RobotHardware hw, Telemetry telemetry, double[] angleOffsets, double imuOffsetDegrees, boolean isMode_DEBUG){
        robot = hw;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DEBUG_MODE = isMode_DEBUG;

        FrontLeft = new SwerveModule(robot.FLT_Motor, robot.FLB_Motor, robot.limitFL, -137, angleOffsets[0], telemetry);
        FrontRight = new SwerveModule(robot.FRT_Motor, robot.FRB_Motor, robot.limitFR, 141, angleOffsets[1], telemetry);
        Back = new SwerveModule(robot.BT_Motor, robot.BB_Motor, robot.limitB, 115, angleOffsets[2], telemetry);
        this.imuOffset = Rotation2d.fromDegrees(imuOffsetDegrees);
    }


    public DrivetrainSubsystem(RobotHardware hw, Telemetry telemetry, boolean isMode_DEBUG){
        robot = hw;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DEBUG_MODE = isMode_DEBUG;

        FrontLeft = new SwerveModule(robot.FLT_Motor, robot.FLB_Motor, robot.limitFL, -137, 0, telemetry);
        FrontRight = new SwerveModule(robot.FRT_Motor, robot.FRB_Motor, robot.limitFR, 141, 0, telemetry);
        Back = new SwerveModule(robot.BT_Motor, robot.BB_Motor, robot.limitB, 115, 0, telemetry);
        imuOffset = Rotation2d.fromDegrees(0);
    }




    public void resetAllEncoders(){
        FrontLeft.resetModuleEncoders();
        FrontRight.resetModuleEncoders();
        Back.resetModuleEncoders();
    }



    public void drive(Vector2d driveVector, double turnSpeed){
        joystickDrive = driveVector;
        joystickTurnSpeed = turnSpeed;
    }




    public void periodic() {

        double driveSpeed = joystickDrive.magnitude();
        double driveAngle = joystickDrive.angle();

        Rotation2d imuAngle = robot.imu.getRotation2d();
        double turnPower;

        if (joystickTurnSpeed == 0 && driveSpeed != 0) {
            double error = targetRobotAngle - imuAngle.getRadians();
            turnPower = robotAngleHoldingKp * error;
        }

        else{
            targetRobotAngle = imuAngle.getRadians();
            turnPower = joystickTurnSpeed * K_rotation;
        }



        telemetry.addData("driveSpeed", driveSpeed);
        telemetry.addData("driveAngle", Math.toDegrees(driveAngle));



        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                driveSpeed * Math.sin(driveAngle) * highestPossibleMotorVelocity,
                driveSpeed * -Math.cos(driveAngle) * highestPossibleMotorVelocity,
                turnPower,
                imuAngle.plus(imuOffset)
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


        robot.FLT_Motor.setVelocity((FrontLeft.turnVelocityTicks + frontLeftModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.FLB_Motor.setVelocity((FrontLeft.turnVelocityTicks - frontLeftModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.FRT_Motor.setVelocity((FrontRight.turnVelocityTicks + frontRightModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.FRB_Motor.setVelocity((FrontRight.turnVelocityTicks - frontRightModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.BT_Motor.setVelocity((Back.turnVelocityTicks + backModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.BB_Motor.setVelocity((Back.turnVelocityTicks - backModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
    }




    public boolean isAtAngle(double angleDegrees, double tolerance){
        double angleError = angleDegrees - robot.imu.getRotation2d().getDegrees();
        return Math.abs(angleError) < tolerance;
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


    public double[] getAllModuleAngleRads(){
        return new double[]{FrontLeft.getAngleRads(), FrontRight.getAngleRads(), Back.getAngleRads()};
    }
}