package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@Config
public class MaintainDistanceExperimental_DriveCommand extends CommandBase {
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double targetDistance = 0; //SOME VALUE YOU TUNE

    DrivetrainSubsystem robot;
    PIDController controller = new PIDController(kp, ki, kd);

    public MaintainDistanceExperimental_DriveCommand(DrivetrainSubsystem robot) {
        this.robot = robot;
        addRequirements(robot);
    }


    @Override
    public void execute() {
        controller.setPID(kp, ki, kd);
        Pose2d joystickInput = robot.getGamepadInput();

        double correction = controller.calculate(0, targetDistance);
        robot.setGamepadInput(new Pose2d(Math.signum(joystickInput.getX()) * correction,
                                  joystickInput.getY(),
                                  joystickInput.getHeading()
        ));
    }
}