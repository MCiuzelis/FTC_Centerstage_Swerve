package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.commands.lowLevelCommands.SetAxonAngleCommand;
import org.firstinspires.ftc.teamcode.commands.lowLevelCommands.SetClawAngleCommand;
import org.firstinspires.ftc.teamcode.commands.lowLevelCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.lowLevelCommands.SetSlideHeightCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.util.function.BooleanSupplier;

public class SetArmToStateCommand extends CommandBase {
    ArmState targetState;
    ArmSubsystem arm;

    BooleanSupplier moveToTransfer = ()-> !(arm.areSlidesDown() && arm.areAxonsCloseToTransferPos());


    public SetArmToStateCommand(ArmSubsystem arm, ArmState state){
        targetState = state;
        this.arm = arm;
        addRequirements(arm);
    }


    @Override
    public void initialize(){
        switch (targetState){
            case PICKUP:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new SetSlideHeightCommand(arm, ArmSubsystem.SLIDE_STATE.PICKUP),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.TRANSFER),
                                                                    new WaitCommand(1000)),
                                        new WaitCommand(0),
                                        moveToTransfer),
                                new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_OPEN),
                                new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.PICKUP),
                                new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.PICKUP)
                        ));
                break;

            case TRANSFER:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                            new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.TRANSFER),
                            new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED),
                            new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.TRANSFER),
                            new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.TRANSFER),
                            new WaitUntilCommand(()-> arm.areAxonsCloseToTransferPos()),
                            new SetSlideHeightCommand(arm, ArmSubsystem.SLIDE_STATE.PICKUP)
                        ));
                break;

            case LOW:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED),
                                new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.LOWPOS),
                                new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.LOWPOS),
                                new SetSlideHeightCommand(arm, ArmSubsystem.SLIDE_STATE.LOW)
                        ));
                break;

            case MID:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED),
                                new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.MIDPOS),
                                new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.MIDPOS),
                                new SetSlideHeightCommand(arm, ArmSubsystem.SLIDE_STATE.MID)
                        ));
                break;

            case HIGH:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED),
                                        new SetAxonAngleCommand(arm, ArmSubsystem.AXON_STATE.HIGHPOS),
                                        new SetClawAngleCommand(arm, ArmSubsystem.CLAW_ANGLE.HIGHPOS),
                                        new SetSlideHeightCommand(arm, ArmSubsystem.SLIDE_STATE.HIGH)
                        ));
                break;
        }
    }



    public enum ArmState{
        PICKUP,
        TRANSFER,
        LOW,
        MID,
        HIGH
    }
}
