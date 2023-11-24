package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

@Config
public class Constants {

    public static double DepositorPickupPosition = 0.925d;
    public static double DepositorDepositPosition = 0.04d;

    public static double DepositorPinHoldPosition = 0.34d;
    public static double DepositorPinReleasePosition = 0;

    public static int LiftLowPosition = 0;
    public static int LiftHighPosition = 1000;

    public static double BroomMotorPower_normal = 0.4d;
    public static double BroomMotorPower_reversed = 0.4d;

    public static double HingeServoPickupPos = 1;
    public static double HingeServoTransferPos = 0;
    public static int MaxAllowedLiftError = 20;

    public static double LiftKp = 0.003d;
    public static double LiftKi = 0.0d;
    public static double LiftKd = 0.0d;
    public static double LiftKf = 0.0d;

    public static double planeLaunchPosition = 0.5;
    public static double planeLockPosition = 0;


    //swerve module offsets from robot center:
    public static Translation2d frontLeftLocation = new Translation2d(0.0837158, 0.145);
    public static Translation2d frontRightLocation = new Translation2d(0.0837158, -0.145);
    public static Translation2d backLocation = new Translation2d(-0.1674315, 0);

    //total gear ratio: 28 - small helical, 61 - big helical
    public static double ticksInOneRad = (((((1d + (46d / 17d))) * (1d + (46d / 17d))) * 28d) * 61d / (26d * 2d * Math.PI));
    public static double highestPossibleMotorVelocity = 2350;
    public static double maxTurningVelocity = highestPossibleMotorVelocity * 0.93f;

    // turning velocity coefficient
    public final static double K_rotation = 10000;

    //multiplier all velocities by this value; if goofy, make small for troubleshooting (0; 1]
    public final static double finalAllMotorVelocityMultiplier = 1;

    public static double DriveBaseTurnKp = 1800;    //turning velocity PID constants
    public static double DriveBaseTurnKi = 50;
    public static double DriveBaseTurnKd = 0.5;

    public static double AutonomousTurningKp = 0;

    public static double fastCalibrationSpeed = 550;
    public static double slowCalibrationSpeed = 250;
}