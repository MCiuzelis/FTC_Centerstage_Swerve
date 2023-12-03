package org.firstinspires.ftc.teamcode.commands.SequentialCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.InstantCommands.SetArmToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.InstantCommands.SetClawAngleCommand;
import org.firstinspires.ftc.teamcode.commands.InstantCommands.SetSlidesToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.InstantCommands.UpdateClawStateCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetArmToDepositPositionCommand extends SequentialCommandGroup {
    public SetArmToDepositPositionCommand(ArmSubsystem arm, ArmSubsystem.SLIDE_TARGET_POSITION SLIDE_POS){
        addCommands(
                    new UpdateClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)
                            .andThen(new SetSlidesToPositionCommand(arm, SLIDE_POS))
                            .alongWith(new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.DEPOSIT))
                            .alongWith(new SetArmToPositionCommand(arm, ArmSubsystem.ARM_TARGET_POSITION.DEPOSIT))

        );
    }
}