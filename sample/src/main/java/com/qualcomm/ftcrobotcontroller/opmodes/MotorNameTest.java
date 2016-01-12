package com.qualcomm.ftcrobotcontroller.opmodes;

import com.lasarobotics.library.controller.ButtonState;
import com.lasarobotics.library.controller.Controller;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Test the motors
 */
public class MotorNameTest extends OpMode {
    DcMotor leftFront, leftBack, rightBack, rightFront, arm, armEnd;
    Servo servo;
    Controller one;

    public static final double ON_POWER = 0.2;
    private int servoAmount = 0;

    @Override
    public void stop() {

    }
    @Override
    public void loop() {
        if(one.a == 3.0)
            leftFront.setPower(ON_POWER);
        else
            leftFront.setPower(0);
        if(one.b == 3.0)
            leftBack.setPower(ON_POWER);
        else
            leftBack.setPower(0);
        if(one.x == 3.0)
            rightFront.setPower(ON_POWER);
        else
            rightFront.setPower(0);
        if(one.y == 3.0)
            rightBack.setPower(ON_POWER);
        else
            rightBack.setPower(0);
        /*if(one.dpad_down == 3.0)
            arm.setPower(ON_POWER);
        else
            arm.setPower(0);
        if(one.dpad_up == 3.0)
            armEnd.setPower(ON_POWER);
        else
            armEnd.setPower(0);
        if(one.dpad_left == 3.0) {
            if(--servoAmount < 0) {
                servoAmount = 0;
            }
        }
        if(one.dpad_right == 3.0) {
            if(++servoAmount > 255) {
                servoAmount = 0;
            }
        }
        servo.setPosition(servoAmount);*/
    }
    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        //arm = hardwareMap.dcMotor.get("arm");
        //armEnd = hardwareMap.dcMotor.get("armEnd");
        //servo = hardwareMap.servo.get("servo");
        one = new Controller(gamepad1);
    }
}
