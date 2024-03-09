package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.CalibrationTransfer;

@Photon
@TeleOp(name = "PlaneCalibration", group = "OpMode")
public class axon_calibration extends OpMode {
    //RobotHardware robotHardware;
    Servo Axon;



    @Override
    public void init() {
        //robotHardware = new RobotHardware(hardwareMap);
        //robotHardware.initialiseHardware(telemetry);
        Axon = hardwareMap.get(Servo.class, "CH_Servo_0");
    }





    @Override
    public void loop(){
        double gamepad = gamepad1.left_stick_y;
        //robotHardware.axonLeft.setPosition(gamepad);
        //robotHardware.axonRight.setPosition(gamepad);
        Axon.setPosition(gamepad);
        //robotHardware.clawLeftServo.setPosition(gamepad);
        //robotHardware.clawRightServo.setPosition(gamepad);
        telemetry.addData("axonPosition: ", gamepad);
    }
}
