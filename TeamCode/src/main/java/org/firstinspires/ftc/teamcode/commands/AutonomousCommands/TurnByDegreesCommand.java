package org.firstinspires.ftc.teamcode.commands.AutonomousCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class TurnByDegreesCommand extends SequentialCommandGroup {
    public TurnByDegreesCommand(DrivetrainSubsystem driveTrain, double angleDegrees, double tolerance, double kp){

        addCommands(
                new TurnCommand(driveTrain, angleDegrees, kp)
                        .andThen(new WaitUntilCommand(()->driveTrain.isAtAngle(angleDegrees, tolerance)))
                        .andThen(new InstantCommand(driveTrain::stop))
        );

    }
}