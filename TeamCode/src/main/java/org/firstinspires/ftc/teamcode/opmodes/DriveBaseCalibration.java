package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@TeleOp(name = "GoofyDriveBaseCalibration", group = "OpMode")
public class DriveBaseCalibration extends CommandOpMode {
    public DrivetrainSubsystem drivetrainSubsystem;
    private final RobotHardware robot = RobotHardware.getInstance();


    @Override
    public void initialize() {
        robot.init(hardwareMap, telemetry);
        drivetrainSubsystem = new DrivetrainSubsystem(robot, telemetry, true);
        drivetrainSubsystem.Calibrate();
    }


    public void run(){
    }
}
