package org.firstinspires.ftc.teamcode.commands.InstantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetSlidesToPositionCommand extends InstantCommand {
    public SetSlidesToPositionCommand(ArmSubsystem armSubsystem, ArmSubsystem.SLIDE_TARGET_POSITION LIFT_POSITION){
        super(()-> armSubsystem.update(LIFT_POSITION));
    }
}
