package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class SetIntakeBroomCommand extends InstantCommand {
    public SetIntakeBroomCommand(IntakeSubsystem intake, IntakeSubsystem.BROOM_MOTOR_STATE state){
        super(()-> intake.update(state));
    }
}
