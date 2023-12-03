package org.firstinspires.ftc.teamcode.commands.AutonomousCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.SequentialCommands.SetArmToTransferPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class AutonomousLeftCommand extends SequentialCommandGroup {
    public AutonomousLeftCommand(ArmSubsystem arm, DrivetrainSubsystem driveTrain){
        addCommands(
                new SetArmToTransferPositionCommand(arm)
                        .andThen(new DriveTimeoutCommand(driveTrain, 0.5, 90, 0, 1000))

        );
    }
}