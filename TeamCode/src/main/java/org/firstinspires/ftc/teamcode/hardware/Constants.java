package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

@Config
public class Constants {

    public static double nominalBatteryVoltage = 14.5;


    //swerve module offsets from robot center:
    public static Translation2d frontLeftLocation = new Translation2d(0.07649891, 0.1325);
    public static Translation2d frontRightLocation = new Translation2d(0.07649891, -0.1325);
    public static Translation2d backLocation = new Translation2d(-0.15299782, 0);

    //total gear ratio: 28 - small helical, 61 - big helical
    public static double ticksInOneRad = (((((1d + (46d / 17d))) * (1d + (46d / 17d))) * 28d) * 52d / (25d * 2d * Math.PI));
    public static double highestPossibleMotorVelocity = 2390;
    public static double turnVelocityMultiplierAtMaxSpeed = 0.6;
    public static double turnVelocityMultiplierCutoffValue = 0.6;
    //public static double maxTurningVelocity = highestPossibleMotorVelocity * 0.9f;

    // turning velocity coefficient
    public static double K_rotation = 7000;

    //multiplier all velocities by this value; if goofy, make small for troubleshooting (0; 1]
    public static double finalAllMotorVelocityMultiplier = 1;

    //DriveBase turning pid:
    public static double DriveBaseTurnKp = 0.45;
    public static double DriveBaseTurnKi = 0;
    public static double DriveBaseTurnKd = 0.002;
    public static double DriveBaseTurnKs = 0.002;
    public static double DriveBaseTurnMaxIntegralSum = 0.01;
    public static double DriveBaseTurnStabilityThreshold = 0.1;
    public static double DriveBaseTurnLowPassGain = 0.8;

    public static double DriveBaseDriveKp = 0.0001;
    public static double DriveBaseDriveKi = 0;
    public static double DriveBaseDriveKd = 0;
    public static double DriveBaseDriveKv = 0.000424;
    public static double DriveBaseDriveKs = 0;

    public static double DriveBaseDriveMaxIntegralSum = 999999;
    public static double DriveBaseDriveStabilityThreshold = 999999;
    public static double DriveBaseDriveLowPassGain = 0;

    public static double DriveBaseMaxDriveAcceleration = 1000;


    public static double robotAngleHoldingKp = 2500;
    public static double robotAngleAllowedErrorRads = Math.toRadians(1.5);

    //Calibration velocities:
    public static double fastCalibrationSpeed = 550;
    public static double slowCalibrationSpeed = 70;



    //Plane
    public static double planeLockPosition = 0.52;
    public static double planeLaunchPosition = 0.55;


    //-----------ARM--------------
    public static double clawClosedPosition = 0;
    public static double clawOpenPosition = 0.75;

    public static double clawPickupPos = 0.4;
    public static double clawLowPosPos = 1;
    public static double clawTransferPos = 0;
    public static double clawMidPosPosition = 0.85;
    public static double clawHighPosPosition = 0.7;

    public static double slidePickupPos = 0;
    public static double slideLowPos = 50;
    public static double slideMidPos = 650;
    public static double slideHighPos = 1100;
}