package org.firstinspires.ftc.teamcode.commands.SequentialCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.InstantCommands.ReleasePixels;
import org.firstinspires.ftc.teamcode.commands.InstantCommands.UpdateClawStateCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ReleasePixelCommand extends SequentialCommandGroup {
    public ReleasePixelCommand(ArmSubsystem arm){
        new UpdateClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_OPEN)
                .alongWith(new ReleasePixels(arm)
                );
    }
}