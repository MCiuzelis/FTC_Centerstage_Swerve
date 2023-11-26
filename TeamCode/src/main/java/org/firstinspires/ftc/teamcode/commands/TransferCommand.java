package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.ams.AMSColorSensor;

import org.firstinspires.ftc.teamcode.subsystems.DepositorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(IntakeSubsystem intakeSubsystem, DepositorSubsystem depositorSubsystem){
        addCommands(
            new SetLiftPositionCommand(depositorSubsystem, DepositorSubsystem.SLIDE_TARGET_POSITION.SAFETODEPOSITPOS)
                    .alongWith(new SetDepositorPinCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PIN_STATE.RELEASE)
                    .alongWith(new SetIntakeHingePositionCommand(intakeSubsystem, IntakeSubsystem.HINGE_SERVO_TARGET_STATE.CHILLIN))
                    .alongWith(new SetDepositorProbePositionCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PROBE_STATE.PICKUP))
                    .alongWith(new WaitUntilCommand(depositorSubsystem::WithinToleranceSafeToDeposit))
                            .andThen(new SetLiftPositionCommand(depositorSubsystem, DepositorSubsystem.SLIDE_TARGET_POSITION.TRANSFERPOS))
                                .alongWith(new WaitUntilCommand(depositorSubsystem::WithinToleranceTransfer)
                                        .andThen(new SetIntakeHingePositionCommand(intakeSubsystem, IntakeSubsystem.HINGE_SERVO_TARGET_STATE.TRANSFER))
                                        .alongWith(new SetIntakeBroomCommand(intakeSubsystem, IntakeSubsystem.BROOM_MOTOR_STATE.DISABLED))
                                        .alongWith(new WaitUntilCommand(intakeSubsystem::ReadyToLock))
                                                .andThen(new WaitCommand(500))
//                                        .withTimeout(2000)
//                                        .andThen()  //try to get unstuck
//                                        .alongWith(new WaitUntilCommand(intakeSubsystem::IntakeServoStuck))
//                                        .withTimeout(2000)
                                            .andThen(new SetDepositorPinCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PIN_STATE.HOLD)
                                                    .andThen(new WaitCommand(1000))
                                                    .andThen(new SetDepositorProbePositionCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PROBE_STATE.GOOFY))
                                                    .andThen(new WaitCommand(1000))
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
