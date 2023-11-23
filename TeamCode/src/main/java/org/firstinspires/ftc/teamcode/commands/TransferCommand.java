package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.DepositorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(IntakeSubsystem intakeSubsystem, DepositorSubsystem depositorSubsystem){
        addCommands(
            new SetLiftPositionCommand(depositorSubsystem, DepositorSubsystem.SLIDE_TARGET_POSITION.LOWPOS)
                    .alongWith(new SetDepositorPinCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PIN_STATE.RELEASE)
                    .alongWith(new SetIntakeHingePositionCommand(intakeSubsystem, IntakeSubsystem.HINGE_SERVO_TARGET_STATE.PICKUP))
                    .alongWith(new SetDepositorProbePositionCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PROBE_STATE.PICKUP))
                    .alongWith(new WaitUntilCommand(depositorSubsystem::WithinToleranceLow)
                            .andThen(new SetIntakeHingePositionCommand(intakeSubsystem, IntakeSubsystem.HINGE_SERVO_TARGET_STATE.TRANSFER))
                            .alongWith(new WaitUntilCommand(intakeSubsystem::ReadyToLock))
                                .andThen(new SetDepositorPinCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PIN_STATE.HOLD)
                                .alongWith(new SetIntakeHingePositionCommand(intakeSubsystem, IntakeSubsystem.HINGE_SERVO_TARGET_STATE.PICKUP))
                            )
                    )
            )
        );

    }

    @Override
    public void initialize() {
        super.initialize();
    }

}
