package org.firstinspires.ftc.teamcode.commands.SequentialCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.InstantCommands.SetArmToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.InstantCommands.SetClawAngleCommand;
import org.firstinspires.ftc.teamcode.commands.InstantCommands.SetSlidesToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.InstantCommands.UpdateClawStateCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetArmToPickupPositionCommand extends SequentialCommandGroup {
    public SetArmToPickupPositionCommand(ArmSubsystem arm, ArmSubsystem.CLAW_STATE POSITION){
        addCommands(
                new SetSlidesToPositionCommand(arm, ArmSubsystem.SLIDE_TARGET_POSITION.PICKUP_POS)
                        .alongWith(new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.PICKUP))
                        //.andThen(new WaitUntilCommand(arm::hasLiftReachedTargetPosition))
                            .andThen(new SetArmToPositionCommand(arm, ArmSubsystem.ARM_TARGET_POSITION.PICKUP))
                            .alongWith(new UpdateClawStateCommand(arm, POSITION))
        );
    }
}