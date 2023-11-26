package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DepositorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class UnstuckIntakeServoCommand extends SequentialCommandGroup {
    public UnstuckIntakeServoCommand(IntakeSubsystem intake, DepositorSubsystem depositor){
        addCommands(
                new SetDepositorPinCommand(depositor, DepositorSubsystem.DEPOSITOR_PIN_STATE.HOLD)
                        .andThen(new SetIntakeHingePositionCommand(intake, IntakeSubsystem.HINGE_SERVO_TARGET_STATE.CHILLIN))
                        .withTimeout(1000)
                        .andThen(new SetDepositorPinCommand(depositor, DepositorSubsystem.DEPOSITOR_PIN_STATE.RELEASE))
                        .alongWith(new SetIntakeHingePositionCommand(intake, IntakeSubsystem.HINGE_SERVO_TARGET_STATE.TRANSFER))
        );
    }
}
