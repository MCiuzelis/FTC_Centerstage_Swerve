package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Localizer;

@Config
public class DriveToPositionExperimental_DriveCommand extends CommandBase {

    Localizer localizer;
    DrivetrainSubsystem swerve;
    Pose2d targetPosition;

    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;

    public static double headingKp = 0;
    public static double headingKi = 0;
    public static double headingKd = 0;

    PIDController translationController = new PIDController(kp, ki, kd);
    PIDController headingController = new PIDController(headingKp, headingKi, headingKd);


    public DriveToPositionExperimental_DriveCommand(DrivetrainSubsystem swerve, Localizer localizer, Pose2d targetPosition) {
        this.swerve = swerve;
        this.localizer = localizer;
        this.targetPosition = targetPosition;
        addRequirements(swerve);
    }


    @Override
    public void execute() {
        translationController.setPID(kp, ki, kd);
        headingController.setPID(headingKp, headingKi, headingKd);

        localizer.updateOdometry();
        Pose2d currentPosition = localizer.getFieldCentricPosition();

        double correctionX = translationController.calculate(currentPosition.getX(), targetPosition.getX());
        double correctionY = translationController.calculate(currentPosition.getY(), targetPosition.getY());
        double headingCorrection = headingController.calculate(currentPosition.getHeading(), targetPosition.getHeading());

        swerve.setGamepadInput(new Pose2d(correctionX, correctionY, headingCorrection));
    }
}