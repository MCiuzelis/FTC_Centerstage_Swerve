package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetClawAngleCommand extends SequentialCommandGroup {
    public SetClawAngleCommand(ArmSubsystem armSubsystem, ArmSubsystem.CLAW_ANGLE CLAW_ANGLE){
        armSubsystem.update(CLAW_ANGLE);
    }
}
