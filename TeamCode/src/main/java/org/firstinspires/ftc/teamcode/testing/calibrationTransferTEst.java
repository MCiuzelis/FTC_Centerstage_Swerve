package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@Disabled
@TeleOp(name = "calibration transfer test", group = "OpMode")
public class calibrationTransferTEst extends CommandOpMode {
    CalibrationTransfer file = new CalibrationTransfer(telemetry);
    RobotHardware hardware = new RobotHardware(hardwareMap);
    DrivetrainSubsystem swerve = new DrivetrainSubsystem(hardware, telemetry, false, false);



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