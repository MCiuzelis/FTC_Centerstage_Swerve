package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@Disabled
@TeleOp(name = "GoofyDriveBaseCalibrationTesting", group = "OpMode")
public class DriveBaseCalibration extends LinearOpMode {
    DrivetrainSubsystem drivetrainSubsystem;
    RobotHardware robot = new RobotHardware(hardwareMap);



    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialiseHardware(telemetry);
        drivetrainSubsystem = new DrivetrainSubsystem(robot, telemetry, true);
        drivetrainSubsystem.calibrate();
        waitForStart();
    }
}
