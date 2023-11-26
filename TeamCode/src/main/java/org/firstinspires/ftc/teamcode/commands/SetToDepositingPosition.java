package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.DepositorSubsystem;

public class SetToDepositingPosition extends SequentialCommandGroup {

    public SetToDepositingPosition(DepositorSubsystem depositorSubsystem, DepositorSubsystem.SLIDE_TARGET_POSITION targetPosition){
        addCommands(
                new SetLiftPositionCommand(depositorSubsystem, targetPosition)
                        .alongWith(new WaitUntilCommand(depositorSubsystem::WithinToleranceSafeToDeposit))
                        .andThen(new SetDepositorProbePositionCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PROBE_STATE.DEPOSIT))
        );
    }
}
