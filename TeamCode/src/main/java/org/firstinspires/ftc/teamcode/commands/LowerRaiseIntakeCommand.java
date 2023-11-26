package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class LowerRaiseIntakeCommand extends SequentialCommandGroup {
    public LowerRaiseIntakeCommand(IntakeSubsystem intake, boolean shouldRaise){
        if (shouldRaise){
            addCommands(new SetIntakeBroomCommand(intake, IntakeSubsystem.BROOM_MOTOR_STATE.DISABLED)
                    .alongWith(new SetIntakeHingePositionCommand(intake, IntakeSubsystem.HINGE_SERVO_TARGET_STATE.CHILLIN)));
        }
        else {
            addCommands(
                    new SetIntakeBroomCommand(intake, IntakeSubsystem.BROOM_MOTOR_STATE.ACTIVE)
                            .alongWith(new SetIntakeHingePositionCommand(intake, IntakeSubsystem.HINGE_SERVO_TARGET_STATE.PICKUP))
            );
        }
    }
}
