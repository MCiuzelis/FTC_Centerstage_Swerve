package org.firstinspires.ftc.teamcode.commands.AutonomousCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.SequentialCommands.SetArmToTransferPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class AutonomousMiddleCommand extends SequentialCommandGroup {
    public AutonomousMiddleCommand(ArmSubsystem arm, DrivetrainSubsystem driveTrain){
        addCommands(
                new SetArmToTransferPositionCommand(arm)
                        .andThen(new DriveTimeoutCommand(driveTrain, 0.5, 90, 0, 1000))
                        .andThen(new TurnByDegreesCommand(driveTrain, 90, 2, 0.001))
        );
    }
}