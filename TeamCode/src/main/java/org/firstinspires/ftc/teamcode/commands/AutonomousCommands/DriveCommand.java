package org.firstinspires.ftc.teamcode.commands.AutonomousCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class DriveCommand extends InstantCommand {
    public DriveCommand(DrivetrainSubsystem driveTrain, double speedMultiplier, double angleDegrees, double turnSpeed){

        super(()->driveTrain.Drive(new Vector2d(speedMultiplier * Math.cos(Math.toRadians(angleDegrees)), speedMultiplier * Math.sin(Math.toRadians(angleDegrees))), turnSpeed));

    }
}