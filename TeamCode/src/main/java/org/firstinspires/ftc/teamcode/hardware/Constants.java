package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

@Config
public class Constants {


    //Lift position constants:
    public static int LiftPickupPosition = 0;
    public static int LiftLowPosition = 100;
    public static int LiftMidPosition = 290;
    public static int LiftHighPosition = 700;


    //Claw angle constants
    public static double ClawAngleTransferPosition = 1;
    public static double ClawAnglePickupPosition = 0.62;
    public static double ClawAngleDepositPosition = 0.55;


    //Claw grab & release constants:
    public static double ClawClosedPosition = 1;
    public static double ClawOpenPosition = 0.5;

    //Lift pid:
    public static double LiftKp = 0.01;
    public static double LiftKi = 0.008;
    public static double LiftKd = 0.00005;
    public static double LiftKf = 0.001;

    public static double LiftPowerClamp = 0.4;

    //Arm pickup and deposit positions:
    public static int armMotorDepositPosition = 440;
    public static int armMotorTransferPosition = 100;
    public static int armMotorPickupPosition = 0;

    public static double armMotorTicksInOneRad = (28d * 76d * 76d * 84d) / (21d * 21d * 29d * 2d * Math.PI);

    //Arm pid:
    public static double ArmKp = -0.0025;
    public static double ArmKi = 0.0025;
    public static double ArmKd = 0;
    public static double ArmKf = 0.02;

    public static double ArmStartAngle = Math.toRadians(20);
    public static double ArmPowerClamp = 0.3;

    //Plane servo positions:
    public static double planeLaunchPosition = 0.55;
    public static double planeLockPosition = 0.47;


    //swerve module offsets from robot center:
    public static Translation2d frontLeftLocation = new Translation2d(0.0837158, 0.145);
    public static Translation2d frontRightLocation = new Translation2d(0.0837158, -0.145);
    public static Translation2d backLocation = new Translation2d(-0.1674315, 0);

    //total gear ratio: 28 - small helical, 61 - big helical
    public static double ticksInOneRad = (((((1d + (46d / 17d))) * (1d + (46d / 17d))) * 28d) * 61d / (26d * 2d * Math.PI));
    public static double highestPossibleMotorVelocity = 2390;
    public static double maxTurningVelocity = highestPossibleMotorVelocity * 0.9f;

    // turning velocity coefficient
    public static double K_rotation = 5000;

    //multiplier all velocities by this value; if goofy, make small for troubleshooting (0; 1]
    public static double finalAllMotorVelocityMultiplier = 1;

    //DriveBase turning pid:
    public static double DriveBaseTurnKp = 1600;
    public static double DriveBaseTurnKi = 40;
    public static double DriveBaseTurnKd = 20;

    public static double DriveBaseDriveKs = 20;
    public static double DriveBaseDriveKv = 2;
    public static double DriveBaseDriveKa = 0.1;

    //robot angle holding and smoothing constants:
    public static double robotAngleHoldingKp = 8000;

    //Calibration velocities:
    public static double fastCalibrationSpeed = 550;
    public static double slowCalibrationSpeed = 250;
}