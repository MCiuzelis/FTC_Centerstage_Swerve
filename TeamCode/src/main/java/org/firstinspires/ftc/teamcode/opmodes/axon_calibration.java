package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.CalibrationTransfer;

@TeleOp(name = "axon", group = "OpMode")
public class axon_calibration extends OpMode {
    Servo axon;



    @Override
    public void init() {
        axon = hardwareMap.get(Servo.class, "EH_Servo_0");
    }





    @Override
    public void loop(){
        axon.setPosition(gamepad1.left_stick_y);
    }
}
