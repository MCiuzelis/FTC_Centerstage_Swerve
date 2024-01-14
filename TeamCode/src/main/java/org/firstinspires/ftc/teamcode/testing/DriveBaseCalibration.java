package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@Disabled
@TeleOp(name = "GoofyDriveBaseCalibration", group = "OpMode")
public class DriveBaseCalibration extends CommandOpMode {
    public DrivetrainSubsystem drivetrainSubsystem;
    RobotHardware robot = new RobotHardware(hardwareMap);


    @Override
    public void initialize() {
        robot.initialiseHardware(telemetry);
        drivetrainSubsystem = new DrivetrainSubsystem(robot, telemetry, true);
        drivetrainSubsystem.calibrate();
    }


    public void run(){
    }
}
