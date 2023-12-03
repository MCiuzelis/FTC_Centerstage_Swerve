package org.firstinspires.ftc.teamcode.commands.InstantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetClawAngleCommand extends InstantCommand {
    public SetClawAngleCommand(ArmSubsystem armSubsystem, ArmSubsystem.CLAW_ANGLE CLAW_ANGLE){
        super(()-> armSubsystem.update(CLAW_ANGLE));
    }
}
