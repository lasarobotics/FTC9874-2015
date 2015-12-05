package com.qualcomm.ftcrobotcontroller.opmodes;

import com.lasarobotics.library.controller.Controller;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Russell on 12/5/2015.
 */
public class EasyAutonomous9874 extends OpMode {
    Controller one = new Controller(gamepad1);
    DcMotor leftBack, rightBack, leftFront, rightFront;

    @Override
    public void init() {
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
    }

    int speedy = 0;
    @Override
    public void loop() {
        speedy++;
        if(speedy < 1000) {
            leftBack.setPower(0.2);
            leftFront.setPower(0.2);
            rightBack.setPower(0.2);
            rightFront.setPower(0.2);
        } else {
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        }
        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
            //do nothing
        }
    }

    @Override
    public void stop() {
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    @Override
    public void start() {

    }
}
