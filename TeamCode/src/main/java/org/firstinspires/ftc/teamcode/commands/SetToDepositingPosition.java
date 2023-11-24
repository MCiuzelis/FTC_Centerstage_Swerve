package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DepositorSubsystem;

public class SetToDepositingPosition extends SequentialCommandGroup {

    public SetToDepositingPosition(DepositorSubsystem depositorSubsystem){
        addCommands(
                new SetLiftPositionCommand(depositorSubsystem, DepositorSubsystem.SLIDE_TARGET_POSITION.HIGHPOS)
                        .alongWith(new SetDepositorProbePositionCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PROBE_STATE.DEPOSIT))
        );
    }
}
