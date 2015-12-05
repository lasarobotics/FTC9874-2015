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
        if(one.a == ButtonState.HELD)
            leftFront.setPower(0.5);
        if(one.b == ButtonState.HELD)
            leftBack.setPower(0.5);
        if(one.x == ButtonState.HELD)
            rightFront.setPower(0.5);
        if(one.y == ButtonState.HELD)
            rightBack.setPower(0.5);
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
