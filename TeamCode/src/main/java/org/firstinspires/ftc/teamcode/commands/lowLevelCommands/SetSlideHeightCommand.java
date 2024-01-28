package org.firstinspires.ftc.teamcode.commands.lowLevelCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetSlideHeightCommand extends InstantCommand {
    public SetSlideHeightCommand (ArmSubsystem arm, ArmSubsystem.SLIDE_STATE height){
        super(()-> arm.update(height));
    }
}
