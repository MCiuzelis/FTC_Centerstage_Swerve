package org.firstinspires.ftc.teamcode.opmodes;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

//@Disabled
@Photon
@TeleOp(name = "GoofyDriveBaseCalibrationTesting", group = "OpMode")
public class DriveBaseCalibration extends LinearOpMode {
    DrivetrainSubsystem drivetrainSubsystem;
    RobotHardware robot;



    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
        robot.initialiseHardware(telemetry);

        drivetrainSubsystem = new DrivetrainSubsystem(robot, telemetry, true, false);
        drivetrainSubsystem.calibrate();
        while (!drivetrainSubsystem.areModulesCalibrated()){
            sleep(1);
            drivetrainSubsystem.updateModuleAngles();
        }
        drivetrainSubsystem.resetAllEncoders();
        drivetrainSubsystem.resetImuOffset();
        waitForStart();

        while (opModeIsActive()){
            robot.getCalibrationSensorTelemetry();
            telemetry.update();
        }
    }
}
