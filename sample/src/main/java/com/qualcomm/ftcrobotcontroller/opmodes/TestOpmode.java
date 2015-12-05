package com.qualcomm.ftcrobotcontroller.opmodes;

import com.lasarobotics.library.controller.Controller;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Russell on 12/5/2015.
 */
public class TestOpmode extends OpMode {
    Controller one = new Controller(gamepad1);
    DcMotor leftBack, rightBack, leftFront, rightFront;

    @Override
    public void init() {
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
    }

    int swap = 0;

    @Override
    public void loop() {
        swap++;
        if(swap > 400) {
            swap = 0;
            telemetry.addData("INFO", "Reset...");
        } else if(swap > 300) {
            leftBack.setPower(0.1);
            leftFront.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            telemetry.addData("INFO", "LEFT BACK");
        } else if(swap > 200) {
            telemetry.addData("INFO", "LEFT FRONT");
            leftFront.setPower(0.1);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        } else if(swap > 100) {
            telemetry.addData("INFO", "RIGHT BACK");
            rightBack.setPower(0.1);
            leftBack.setPower(0);
            rightFront.setPower(0);
            leftFront.setPower(0);
        } else {
            telemetry.addData("INFO", "RIGHT FRONT");
            rightFront.setPower(0.1);
            rightBack.setPower(0);
            leftFront.setPower(0);
            leftBack.setPower(0);
        }
    }

    @Override
    public void start() {

    }
}
