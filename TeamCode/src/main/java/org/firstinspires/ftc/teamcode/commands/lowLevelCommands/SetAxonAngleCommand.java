package org.firstinspires.ftc.teamcode.commands.lowLevelCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetAxonAngleCommand extends InstantCommand {
    public SetAxonAngleCommand(ArmSubsystem arm, ArmSubsystem.AXON_STATE state){
        super(()-> arm.update(state));
    }
}
