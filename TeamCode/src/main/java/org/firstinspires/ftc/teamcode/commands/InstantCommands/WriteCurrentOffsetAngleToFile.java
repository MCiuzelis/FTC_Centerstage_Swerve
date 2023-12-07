package org.firstinspires.ftc.teamcode.commands.InstantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class WriteCurrentOffsetAngleToFile extends InstantCommand {
    public WriteCurrentOffsetAngleToFile(CalibrationTransfer file, RobotHardware robot, DrivetrainSubsystem driveTrain){
        super(()-> file.PushCalibrationData(robot.imu.getRotation2d().getDegrees(), driveTrain.getAllModuleAngleRads()));
    }
}
