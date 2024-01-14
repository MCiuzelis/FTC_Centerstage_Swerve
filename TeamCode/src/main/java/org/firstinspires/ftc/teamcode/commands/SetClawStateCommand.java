package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetClawStateCommand extends InstantCommand {
    public SetClawStateCommand(ArmSubsystem arm, ArmSubsystem.CLAW_STATE state){
        super(() -> arm.update(state));
    }
}
