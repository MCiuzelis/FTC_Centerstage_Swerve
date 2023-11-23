package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class SetIntakeHingePositionCommand extends InstantCommand {
    public SetIntakeHingePositionCommand(IntakeSubsystem intake, IntakeSubsystem.HINGE_SERVO_TARGET_STATE state) {
        super(()->intake.update(state));
    }
}
