package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

@Config
public class Constants {

    public static double DepositorPickupPosition = 0.058;
    public static double DepositorGoofyPosition = 0.35;
    public static double DepositorDepositPosition = 1;

    public static double DepositorPinHoldPosition = 0.37;
    public static double DepositorPinReleasePosition = 0;

    public static int LiftTransferPosition = 0;
    public static int LiftSafeToDepositPosition = 300;
    public static int LiftLowPosition = 500;
    public static int LiftMidPosition = 900;
    public static int LiftHighPosition = 1300;

    public static double BroomMotorPower_normal = 0.4;
    public static double BroomMotorPower_reversed = 0.4d;

    public static double HingeServoPickupPos = 0.28;
    public static double HingeServoTransferPos = 95;
    public static double HingeServoChillPos = 0.5;

    public static int MaxAllowedLiftError = 10;

    public static double LiftKp = 0.003;
    public static double LiftKi = 0.0d;
    public static double LiftKd = 0.0d;
    public static double LiftKf = 0.0d;

    public static double planeLaunchPosition = 0.6;
    public static double planeLockPosition = 0;


    //swerve module offsets from robot center:
    public static Translation2d frontLeftLocation = new Translation2d(0.0837158, 0.145);
    public static Translation2d frontRightLocation = new Translation2d(0.0837158, -0.145);
    public static Translation2d backLocation = new Translation2d(-0.1674315, 0);

    //total gear ratio: 28 - small helical, 61 - big helical
    public static double ticksInOneRad = (((((1d + (46d / 17d))) * (1d + (46d / 17d))) * 28d) * 61d / (26d * 2d * Math.PI));
    public static double highestPossibleMotorVelocity = 2390;
    public static double maxTurningVelocity = highestPossibleMotorVelocity * 0.95f;

    // turning velocity coefficient
    public final static double K_rotation = 10000;

    //multiplier all velocities by this value; if goofy, make small for troubleshooting (0; 1]
    public final static double finalAllMotorVelocityMultiplier = 0.2;

    public static double DriveBaseTurnKp = 1900;    //turning velocity PID constants
    public static double DriveBaseTurnKi = 30;
    public static double DriveBaseTurnKd = 25;

    public static double robotAngleHoldingKp = 0;

    public static double fastCalibrationSpeed = 550;
    public static double slowCalibrationSpeed = 250;
}