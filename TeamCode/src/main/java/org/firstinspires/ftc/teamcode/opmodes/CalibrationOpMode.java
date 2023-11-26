package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@TeleOp(name = "GoofyCalibration", group = "OpMode")
public class CalibrationOpMode extends CommandOpMode {
    public DrivetrainSubsystem drivetrainSubsystem;
    private final RobotHardware robot = RobotHardware.getInstance();



    @Override
    public void initialize() {

        robot.init(hardwareMap, telemetry);
        drivetrainSubsystem = new DrivetrainSubsystem(robot, telemetry, true);
        drivetrainSubsystem.Calibrate();

        robot.depositorPinServo.setPosition(0);

    }


    public void run(){
        if (gamepad1.triangle){
            //robot.depositorLeftServo.setPosition(1);
            //robot.depositorRightServo.setPosition(1);
            //robot.planeServo.setPosition(0);
            robot.intakeHingeServo.setPosition(0.3);
        }
        telemetry.addData("IsPressed?", robot.limitIntake.isPressed());
        if (gamepad1.circle){
            robot.intakeHingeServo.setPosition(0.7);
            //robot.planeServo.setPosition(0.7);
            //robot.depositorLeftServo.setPosition(0);
            //robot.depositorRightServo.setPosition(0);
        }
        telemetry.addData("slide ticks", robot.depositorSlideMotor.getCurrentPosition());
        telemetry.update();
    }
}
