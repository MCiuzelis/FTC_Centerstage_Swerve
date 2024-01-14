package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetClawAngleCommand extends InstantCommand {
    public SetClawAngleCommand(ArmSubsystem arm, ArmSubsystem.CLAW_ANGLE angle){
        super(()-> arm.update(angle));
    }
}
