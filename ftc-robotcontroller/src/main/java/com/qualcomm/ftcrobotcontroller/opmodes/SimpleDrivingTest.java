package com.qualcomm.ftcrobotcontroller.opmodes;

import com.lasarobotics.library.controller.Controller;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SimpleDrivingTest extends OpMode {
    Controller one = new Controller(gamepad1);

    DcMotor leftBack, rightBack, leftFront, rightFront;

    @Override
    public void init() {
        //Assign variables to motors
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
    }

    @Override
    public void loop() {
        one.update(gamepad1);
    }
}
