package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.hardware.Constants.planeLaunchPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.SetArmPositionCommand;
import org.firstinspires.ftc.teamcode.commands.SetClawAngleCommand;
import org.firstinspires.ftc.teamcode.commands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.hardware.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.hardware.GamePad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@Disabled
@Config
@TeleOp(name = "calibration transfer test", group = "OpMode")
public class calibrationTransferTEst extends CommandOpMode {
    CalibrationTransfer file = new CalibrationTransfer(telemetry);
    RobotHardware hardware = new RobotHardware(hardwareMap);
    DrivetrainSubsystem swerve = new DrivetrainSubsystem(hardware, telemetry, false);



    @Override
    public void initialize() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }



    @Override
    public void run(){

        double [] test = file.pullModuleAngleOffsets();


        telemetry.addData("0", test[0]);
        telemetry.addData("1", test[1]);
        telemetry.addData("2", test[2]);

        telemetry.update();
        //armSubsystem.periodic();


    }
}