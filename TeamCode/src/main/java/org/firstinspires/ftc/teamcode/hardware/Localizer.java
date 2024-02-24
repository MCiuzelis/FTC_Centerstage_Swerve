package org.firstinspires.ftc.teamcode.hardware;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Localizer {

    public static double sensorOffsetX = 0;
    public static double sensorOffsetY = 0;

    public static double velocityLowPassGain = 0.5;
    public static double trackBallUnitsInOneCm = 1;

    double imuAdditiveOffset = 0;
    double prevImuAngle = 0;

    double counter = 0;
    double distanceSum = 0;

    double prevTime = 0;
    Pose2d prevRobotPosition = new Pose2d();
    Pose2d robotCentricPosition = new Pose2d();
    Pose2d robotCentricVelocity = new Pose2d();
    Pose2d fieldCentricPosition = new Pose2d();

    ElapsedTime timer = new ElapsedTime();
    RobotHardware robot;
    Telemetry telemetry;

    LowPassFilter xFilter = new LowPassFilter(velocityLowPassGain);
    LowPassFilter yFilter = new LowPassFilter(velocityLowPassGain);
    LowPassFilter headingFilter = new LowPassFilter(velocityLowPassGain);


    public Localizer (RobotHardware robot, Telemetry telemetry, boolean resetSensor){
        this.robot = robot;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (resetSensor) robot.trackBall.reset();
    }


    public Pose2d getRobotCentricPosition(){
        return new Pose2d(robotCentricPosition.getY(), robotCentricPosition.getX(), robotCentricPosition.getHeading());
    }

    public Pose2d getRobotCentricVelocity(){
        return new Pose2d(robotCentricVelocity.getY(), robotCentricVelocity.getX(), robotCentricVelocity.getHeading());
    }

    public Pose2d getFieldCentricPosition() {
        return new Pose2d(fieldCentricPosition.getY(), fieldCentricPosition.getX(), fieldCentricPosition.getHeading());
    }

    public void updateOdometry (){
        double imuAngle = robot.imuAngle.getRadians();

        robotCentricPosition = getCurrentPosition(imuAngle);

        double currentTime = timer.seconds();
        double dt = currentTime - prevTime;
        prevTime = currentTime;

        Pose2d positionDelta = new Pose2d(robotCentricPosition.getX() - prevRobotPosition.getX(),
                                          robotCentricPosition.getY() - prevRobotPosition.getY(),
                                      robotCentricPosition.getHeading() - prevRobotPosition.getHeading());

        robotCentricVelocity =  new Pose2d(xFilter.estimate(positionDelta.getX() / dt),
                                    yFilter.estimate(positionDelta.getY() / dt),
                                    headingFilter.estimate(positionDelta.getHeading() / dt));

        fieldCentricPosition = new Pose2d(fieldCentricPosition.getX() + positionDelta.getX() * Math.cos(imuAngle),
                                          fieldCentricPosition.getY() + positionDelta.getY() * Math.sin(imuAngle),
                                             imuAngle);
        prevRobotPosition = robotCentricPosition;
    }

    private Pose2d getCurrentPosition(double imuAngle){
        double x = robot.trackBall.getX() / trackBallUnitsInOneCm;
        double y = -robot.trackBall.getY() / trackBallUnitsInOneCm;

        double absoluteImuAngle = getAbsoluteImuAngle(imuAngle);
        telemetry.addData("absoluteImuAngleDegrees", Math.toDegrees(absoluteImuAngle));

        double compensationX = getRotationalLocalizerOffset(absoluteImuAngle, sensorOffsetX);
        double compensationY = getRotationalLocalizerOffset(absoluteImuAngle, sensorOffsetY);

        telemetry.addData("compensationX", compensationX);
        telemetry.addData("compensationY", compensationY);

        return new Pose2d(x - compensationX, y - compensationY, imuAngle);
    }

    private double getRotationalLocalizerOffset (double imuAngle, double offset){
        return (2d * Math.PI * offset * (imuAngle / (2 * Math.PI)));
    }

    private double getAbsoluteImuAngle (double imuAngleRads) {
        if (imuAngleRads < 0){
            imuAngleRads += 2 * Math.PI;
        }
        double imuAngleDisplacement = imuAngleRads - prevImuAngle;

        if (Math.abs(imuAngleDisplacement) > Math.PI) {
            imuAdditiveOffset -= Math.signum(imuAngleDisplacement) * 2 * Math.PI;
        }
        prevImuAngle = imuAngleRads;
        return  imuAngleRads + imuAdditiveOffset;
    }

    public double calculateStartPoseXOffset (){
        return distanceSum / counter;
    }
}