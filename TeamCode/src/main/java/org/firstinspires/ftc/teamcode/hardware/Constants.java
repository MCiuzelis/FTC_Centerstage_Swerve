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
    public static double K_rotation = 6000;

    //multiplier all velocities by this value; if goofy, make small for troubleshooting (0; 1]
    public static double finalAllMotorVelocityMultiplier = 1;

    //DriveBase turning pid:
    public static double DriveBaseTurnKp = 0.45;
    public static double DriveBaseTurnKi = 0.0001;
    public static double DriveBaseTurnKd = 0.002;
    public static double DriveBaseTurnKs = 0.002;
    public static double DriveBaseTurnMaxIntegralSum = 0.01;
    public static double DriveBaseTurnStabilityThreshold = 0.1;
    public static double DriveBaseTurnLowPassGain = 0.8;


    public static double DriveBaseDriveKp = 0.0001;
    public static double DriveBaseDriveKi = 0;
    public static double DriveBaseDriveKd = 0;
    public static double DriveBaseDriveKv = 0.000424;
    //public static double DriveBaseDriveKa = 0;
    public static double DriveBaseDriveKs = 0;

    public static double DriveBaseDriveMaxIntegralSum = 999999;
    public static double DriveBaseDriveStabilityThreshold = 999999;
    public static double DriveBaseDriveLowPassGain = 0;

    public static double DriveBaseMaxDriveAcceleration = 1000;


    public static double robotAngleHoldingKp = 0;
    public static double robotAngleHoldingKpStatic = 0;

    //Calibration velocities:
    public static double fastCalibrationSpeed = 550;
    public static double slowCalibrationSpeed = 70;



    //Plane
    public static double planeLockPosition = 0.47;
    public static double planeLaunchPosition = 0.55;


    //-----------ARM--------------
    public static double ClawClosedPosition = 0;
    public static double ClawOpenPosition = 0.75;

    public static double ClawPickupPosition = 0.55;
    public static double ClawLowPosPosition = 0.68;
    public static double ClawMidPosPosition = 0.625;
    public static double ClawHighPosPosition = 0.65;

    public static double ArmPickupAngle = 0;
    public static double ArmMidPosAngle = 150;
    public static double ArmLowPosAngle = 175;
    public static double ArmHighPosAngle = 165;
    public static double ArmTransferPosAngle = 25;


    private final static double ArmMotorEncoderResolution = ((((1d+(46d/17d))) * (1d+(46d/11d))) * 28d);
    private final static double ArmGearRatio = 22d / 30d;
    public static double ArmTicksInOneRad = ArmMotorEncoderResolution / (ArmGearRatio * Math.PI * 2);
    public static double ArmStartOffsetAngleRads = Math.toRadians(-23);

//    public static double ArmKpSmallError = 0.00243 ;
//    public static double ArmKiSmallError = 0.07418;
//    public static double ArmKdSmallError = 0.0025;

    public static double ArmKpSmallError = 0.005 ;
    public static double ArmKiSmallError = 0.00035;
    public static double ArmKdSmallError = 0.0007;
//
//    public static double ArmKpLargeError = 0.0196 ;
//    public static double ArmKiLargeError = 0.044;
//    public static double ArmKdLargeError = 1000;

    //public static double ArmKCos = 0.2;
    public static double ArmKCos = 0.15;


    //    public static double ArmKa = 0;
//    public static double ArmKv = 0;
    public static double ArmKs = 0.000005;
    public static double ArmPidCap = 0.7;

//    public static double ArmMaxIntegralSum = 0;
//    public static double ArmStabilityThreshold = 0;
//    public static double ArmLowPassGain = 0;
}