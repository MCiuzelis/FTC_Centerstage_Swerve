package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.DepositorSubsystem;

public class SetDepositorProbePositionCommand extends InstantCommand {
    public SetDepositorProbePositionCommand(DepositorSubsystem depositor, DepositorSubsystem.DEPOSITOR_PROBE_STATE state){
        super(()->depositor.update(state));
    }


}
