package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@Photon
@Disabled
@TeleOp(name = "analogTest", group = "OpMode")
public class analogTest extends CommandOpMode {
    AnalogInput analog1;
    AnalogInput analog2;

    @Override
    public void initialize() {
        analog1 = hardwareMap.get(AnalogInput.class, "0");
        analog2 = hardwareMap.get(AnalogInput.class, "1");
    }

    @Override
    public void run(){
        telemetry.addData("0 voltage: ", analog1.getVoltage());
        telemetry.addData("1 voltage: ", analog2.getVoltage());
        telemetry.update();
    }
}
