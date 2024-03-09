package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@Photon
@Disabled
@TeleOp(name = "servoPowerModuleTest", group = "OpMode")
public class servoPowerModule extends OpMode {
    //RobotHardware robotHardware;
    Servo test;



    @Override
    public void init() {
        test = hardwareMap.get(Servo.class, "test");
    }





    @Override
    public void loop(){
        double gamepad = gamepad1.left_stick_y;
        test.setPosition(gamepad);
    }
}
