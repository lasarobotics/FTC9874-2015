package com.qualcomm.ftcrobotcontroller.opmodes;

import com.lasarobotics.library.controller.ButtonState;
import com.lasarobotics.library.controller.Controller;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Will on 12/5/2015.
 */
public class MotorNameTest extends OpMode {
    DcMotor leftFront, leftBack, rightBack, rightFront;
    Controller one;

    @Override
    public void stop() {

    }
    @Override
    public void loop() {
        if(one.a == ButtonState.PRESSED)
            leftFront.setPower(0.5);
        else
            leftFront.setPower(0);
        if(one.b == ButtonState.PRESSED)
            leftBack.setPower(0.5);
        else
            leftBack.setPower(0);
        if(one.x == ButtonState.PRESSED)
            rightFront.setPower(0.5);
        else
            rightFront.setPower(0);
        if(one.y == ButtonState.PRESSED)
            rightBack.setPower(0.5);
        else
            rightBack.setPower(0);
    }
    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        one = new Controller(gamepad1);
    }
}
