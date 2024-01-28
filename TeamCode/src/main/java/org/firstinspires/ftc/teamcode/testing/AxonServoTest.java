package org.firstinspires.ftc.teamcode.testing;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Photon
@Disabled
@TeleOp(name = "AxonTest", group = "OpMode")
public class AxonServoTest extends OpMode {
    Servo axon1, axon0;


    @Override
    public void init() {

        axon0 = hardwareMap.get(Servo.class, "EHservo3");
        //axon0.setDirection(Servo.Direction.REVERSE);
        axon1 = hardwareMap.get(Servo.class, "EHservo2");

    }



    @Override
    public void loop(){

        if (gamepad1.left_bumper){
            axon0.setPosition(0.025);
            axon1.setPosition(0.025);
        }
        else if (gamepad1.right_bumper){
            axon0.setPosition(0.4);
            axon1.setPosition(0.4);
        }
    }
}
