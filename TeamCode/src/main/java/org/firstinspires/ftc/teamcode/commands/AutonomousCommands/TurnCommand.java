package org.firstinspires.ftc.teamcode.commands.AutonomousCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class TurnCommand extends InstantCommand {
    public TurnCommand(DrivetrainSubsystem driveTrain, double kp){
        super(()->driveTrain.drive(new Vector2d(0, 0), kp));
    }
}