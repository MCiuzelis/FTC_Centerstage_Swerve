package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.DepositorSubsystem;

public class SetLiftPositionCommand extends InstantCommand {
    public SetLiftPositionCommand(DepositorSubsystem depositor, DepositorSubsystem.SLIDE_TARGET_POSITION state){
        super(()-> depositor.update(state));
    }
    public SetLiftPositionCommand(DepositorSubsystem depositor, float power, boolean isReversed) {
        super(() -> {
            depositor.update(isReversed ? -power : power);
        });
    }
    public SetLiftPositionCommand(DepositorSubsystem depositor, int offset){
        super(() -> depositor.update(offset));
    }
}
