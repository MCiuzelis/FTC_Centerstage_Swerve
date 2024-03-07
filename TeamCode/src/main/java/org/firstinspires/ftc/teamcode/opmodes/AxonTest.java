package org.firstinspires.ftc.teamcode.opmodes;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@Photon
@Disabled
@TeleOp(name = "ArmServoTest", group = "OpMode")
public class AxonTest extends OpMode {
    //RobotHardware robotHardware;
    Servo AxonLeft, AxonRight;
    AnalogInput analog0;
    AnalogInput analog1;



    @Override
    public void init() {
        //robotHardware = new RobotHardware(hardwareMap);
        //robotHardware.initialiseHardware(telemetry);
        AxonLeft = hardwareMap.get(Servo.class, "0");
        AxonRight = hardwareMap.get(Servo.class, "1");
        AxonRight.setDirection(Servo.Direction.REVERSE);
        analog0 = hardwareMap.get(AnalogInput.class, "an0");
        analog1 = hardwareMap.get(AnalogInput.class, "an1");
    }





    @Override
    public void loop(){
        double gamepad = gamepad1.left_stick_y;
        //robotHardware.axonLeft.setPosition(gamepad);
        //robotHardware.axonRight.setPosition(gamepad);
        AxonLeft.setPosition(gamepad);
        AxonRight.setPosition(gamepad);
        //robotHardware.clawLeftServo.setPosition(gamepad);
        //robotHardware.clawRightServo.setPosition(gamepad);

        telemetry.addData("analog0: ", analog0.getVoltage());
        telemetry.addData("analog1: ", analog1.getVoltage());
        telemetry.addData("axonPosition: ", gamepad);
    }
}
