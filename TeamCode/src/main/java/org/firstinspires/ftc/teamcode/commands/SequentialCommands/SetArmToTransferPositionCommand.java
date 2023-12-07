package org.firstinspires.ftc.teamcode.commands.SequentialCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.InstantCommands.SetArmToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.InstantCommands.SetClawAngleCommand;
import org.firstinspires.ftc.teamcode.commands.InstantCommands.SetSlidesToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.InstantCommands.UpdateClawStateCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetArmToTransferPositionCommand extends SequentialCommandGroup {
    public SetArmToTransferPositionCommand(ArmSubsystem arm){
        addCommands(
                new UpdateClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)
                        .andThen(new WaitCommand(100))
                            .andThen(new SetSlidesToPositionCommand(arm, ArmSubsystem.SLIDE_TARGET_POSITION.PICKUP_POS))
                                    .andThen(new SetArmToPositionCommand(arm, ArmSubsystem.ARM_TARGET_POSITION.TRANSFER))
                                            .andThen(new WaitCommand(300))
                                            .andThen(new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.TRANSFER))

        );
    }
}