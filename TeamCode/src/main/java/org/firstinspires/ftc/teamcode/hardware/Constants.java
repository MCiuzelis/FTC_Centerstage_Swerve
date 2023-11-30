package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

@Config
public class Constants {


    //Lift position constants:
    public static int LiftPickupPosition = 0;
    public static int LiftLowPosition = 500;
    public static int LiftMidPosition = 900;
    public static int LiftHighPosition = 1300;

    //Claw angle constants
    public static int ClawAnglePickupPosition = 0;
    public static int ClawAngleDepositPosition = 0;

    // claw angle servo y = kx + b function set points
    public static double [] firstSetPoint = {0, 0};
    public static double [] secondSetPoint = {0, 0};

    //Claw grab & release constants:
    public static double ClawClosedPosition = 0;
    public static double ClawOpenPosition = 0;

    //Lift pid:
    public static double LiftKp = 0.003;
    public static double LiftKi = 0.0d;
    public static double LiftKd = 0.0d;
    public static double LiftKf = 0.0d;


    //Arm pickup and deposit positions:
    public static int armMotorDepositPosition = 0;
    public static int armMotorPickupPosition = 0;

    //Arm pid:
    public static double ArmKp = 0;
    public static double ArmKi = 0;

    //Plane servo positions:
    public static double planeLaunchPosition = 0.6;
    public static double planeLockPosition = 0;


    //swerve module offsets from robot center:
    public static Translation2d frontLeftLocation = new Translation2d(0.0837158, 0.145);
    public static Translation2d frontRightLocation = new Translation2d(0.0837158, -0.145);
    public static Translation2d backLocation = new Translation2d(-0.1674315, 0);

    //total gear ratio: 28 - small helical, 61 - big helical
    public static double ticksInOneRad = (((((1d + (46d / 17d))) * (1d + (46d / 17d))) * 28d) * 61d / (26d * 2d * Math.PI));
    public static double highestPossibleMotorVelocity = 2390;
    public static double maxTurningVelocity = highestPossibleMotorVelocity * 0.9f;

    // turning velocity coefficient
    public static double K_rotation = 10000;

    //multiplier all velocities by this value; if goofy, make small for troubleshooting (0; 1]
    public static double finalAllMotorVelocityMultiplier = 1;

    //DriveBase turning pid:
    public static double DriveBaseTurnKp = 1600;
    public static double DriveBaseTurnKi = 40;
    public static double DriveBaseTurnKd = 20;

    //robot angle holding and smoothing constants:
    public static double driveVelocitySmoothingRatio = 0.8;
    public static double angleHoldingDelayMs = 460;
    public static double robotAngleHoldingKp = 7000;
    public static double allowedRobotAngleError = Math.toRadians(7);

    //Calibration velocities:
    public static double fastCalibrationSpeed = 550;
    public static double slowCalibrationSpeed = 250;
}