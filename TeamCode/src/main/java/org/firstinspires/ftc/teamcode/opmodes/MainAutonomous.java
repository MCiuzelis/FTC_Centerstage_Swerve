package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ResetDriveEncodersCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@Autonomous(name = "MainAutonomous", group = "OpMode")
public class MainAutonomous extends CommandOpMode {
    public DrivetrainSubsystem drivetrainSubsystem;
    private final RobotHardware robot = RobotHardware.getInstance();



    @Override
    public void initialize() {

        robot.init(hardwareMap, telemetry);
        drivetrainSubsystem = new DrivetrainSubsystem(robot, telemetry, true);
        new ResetDriveEncodersCommand(drivetrainSubsystem);
    }

    @Override
    public void run(){
        super.run();
        drivetrainSubsystem.turnRobotToAngle(10);
    }
}