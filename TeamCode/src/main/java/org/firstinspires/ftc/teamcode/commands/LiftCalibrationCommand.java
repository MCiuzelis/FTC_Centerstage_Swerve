package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DepositorSubsystem;

public class LiftCalibrationCommand extends SequentialCommandGroup {
    public LiftCalibrationCommand(DepositorSubsystem depositor, RobotHardware robot){
        if (robot.depositorSlideMotor.getCurrent(CurrentUnit.AMPS) > 0){

        }
    }
}
