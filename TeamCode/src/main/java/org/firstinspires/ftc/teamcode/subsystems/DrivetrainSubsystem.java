package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.hardware.Constants.K_rotation;
import static org.firstinspires.ftc.teamcode.hardware.Constants.backLocation;
import static org.firstinspires.ftc.teamcode.hardware.Constants.frontLeftLocation;
import static org.firstinspires.ftc.teamcode.hardware.Constants.frontRightLocation;
import static org.firstinspires.ftc.teamcode.hardware.Constants.highestPossibleMotorVelocity;
import static org.firstinspires.ftc.teamcode.hardware.Constants.finalAllMotorVelocityMultiplier;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.GamePad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
public class DrivetrainSubsystem extends SubsystemBase {

    public RobotHardware robot;
    public Telemetry telemetry;
    public boolean DEBUG_MODE;

    SwerveModule FrontLeft, FrontRight, Back;
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLocation);

    public DrivetrainSubsystem(RobotHardware hw, Telemetry telemetry, boolean isMode_DEBUG){
        robot = hw;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DEBUG_MODE = isMode_DEBUG;

        FrontLeft = new SwerveModule(robot.FLT_Motor, robot.FLB_Motor, robot.limitFL, -137);
        FrontRight = new SwerveModule(robot.FRT_Motor, robot.FRB_Motor, robot.limitFR, 141);
        Back = new SwerveModule(robot.BT_Motor, robot.BB_Motor, robot.limitB, 115);

        //Start Multithreaded calibration
        FrontLeft.start();
        FrontRight.start();
        Back.start();
    }

    public void ResetAllEncoders(){
        FrontLeft.resetModuleEncoders();
        FrontRight.resetModuleEncoders();
        Back.resetModuleEncoders();
    }

    public void Drive(GamePad gamePad) {

        Vector2d joystick = gamePad.getJoystickVector();
        double driveSpeed = joystick.magnitude();
        double driveAngle = joystick.angle();
        telemetry.addData("magnitude", driveSpeed);
        telemetry.addData("angle", driveAngle);


        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                driveSpeed * Math.sin(driveAngle) * highestPossibleMotorVelocity,
                driveSpeed * Math.cos(driveAngle) * highestPossibleMotorVelocity,
                gamePad.turnSpeed() * K_rotation,
                                    robot.imu.getRotation2d()
        );

        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState back = moduleStates[2];

        telemetry.addData("frontLeft", frontLeft.speedMetersPerSecond);

        FrontLeft.updateTurningAndDrivingSpeeds(frontLeft.angle, frontLeft.speedMetersPerSecond);
        FrontRight.updateTurningAndDrivingSpeeds(frontRight.angle, frontRight.speedMetersPerSecond);
        Back.updateTurningAndDrivingSpeeds(back.angle, back.speedMetersPerSecond);


        double frontLeftModuleAvailableVelocity = highestPossibleMotorVelocity - Math.abs(FrontLeft.turnVelocityTicks);
        double frontRightModuleAvailableVelocity = highestPossibleMotorVelocity - Math.abs(FrontRight.turnVelocityTicks);
        double backModuleAvailableVelocity = highestPossibleMotorVelocity - Math.abs(Back.turnVelocityTicks);

        double frontLeftModuleAvailableAndRequestedVelocityRatio = Math.abs(frontLeftModuleAvailableVelocity / FrontLeft.driveSpeedMultiplier);
        double frontRightModuleAvailableAndRequestedVelocityRatio = Math.abs(frontRightModuleAvailableVelocity / FrontRight.driveSpeedMultiplier);
        double backModuleAvailableAndRequestedVelocityRatio = Math.abs(backModuleAvailableVelocity / Back.driveSpeedMultiplier);

        telemetry.addData("awailable", frontLeftModuleAvailableVelocity);
        telemetry.addData("multiplier", FrontLeft.driveSpeedMultiplier);

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

        telemetry.addData("final", frontLeftModuleFinalDriveVelocity);

        robot.FLT_Motor.setVelocity((FrontLeft.turnVelocityTicks + frontLeftModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.FLB_Motor.setVelocity((FrontLeft.turnVelocityTicks - frontLeftModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.FRT_Motor.setVelocity((FrontRight.turnVelocityTicks + frontRightModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.FRB_Motor.setVelocity((FrontRight.turnVelocityTicks - frontRightModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.BT_Motor.setVelocity((Back.turnVelocityTicks + backModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
        robot.BB_Motor.setVelocity((Back.turnVelocityTicks - backModuleFinalDriveVelocity) * finalAllMotorVelocityMultiplier);
    }
}