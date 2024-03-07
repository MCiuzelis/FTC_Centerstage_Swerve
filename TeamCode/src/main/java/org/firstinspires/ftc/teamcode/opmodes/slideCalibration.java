package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@Photon
@Disabled
@TeleOp(name = "slideTEst", group = "OpMode")
public class slideCalibration extends CommandOpMode {
    RobotHardware robotHardware;
    ArmSubsystem armSubsystem;

    @Override
    public void initialize() {
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.initialiseHardware(telemetry);
        armSubsystem = new ArmSubsystem(robotHardware, telemetry, true);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run(){
        if (gamepad1.cross) {
            armSubsystem.update(ArmSubsystem.SLIDE_STATE.PICKUP);
        }
        else if (gamepad1.circle){
            armSubsystem.update(ArmSubsystem.SLIDE_STATE.LOW);
        }
        else if (gamepad1.square){
            armSubsystem.update(ArmSubsystem.SLIDE_STATE.MID);
        }
        else if (gamepad1.triangle){
            armSubsystem.update(ArmSubsystem.SLIDE_STATE.HIGH);
        }

        CommandScheduler.getInstance().run();

        telemetry.addData("slideMotorPositionTicks: ", robotHardware.SlideMotor.getCurrentPosition());
        telemetry.update();
    }
}
