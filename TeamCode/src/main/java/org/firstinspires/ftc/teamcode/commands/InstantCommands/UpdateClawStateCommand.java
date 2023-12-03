package org.firstinspires.ftc.teamcode.commands.InstantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class UpdateClawStateCommand extends InstantCommand {
    public UpdateClawStateCommand(ArmSubsystem armSubsystem, ArmSubsystem.CLAW_STATE CLAW_STATE){
        super(()-> armSubsystem.update(CLAW_STATE));
    }
}