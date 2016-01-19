package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Russell on 1/16/2016.
 */
public class ControllerlessMotorTest extends OpMode {
    public static final int NUM_TICKS = 200;
    public static final double POWER = 0.2;

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
        if(speedy < NUM_TICKS) {
            leftBack.setPower(POWER);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
            telemetry.addData("Wheel", "leftBack");
        } else if(speedy < NUM_TICKS*2) {
            leftFront.setPower(POWER);
            rightBack.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            telemetry.addData("Wheel", "leftFront");
        } else if(speedy < NUM_TICKS*3) {
            rightBack.setPower(POWER);
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            telemetry.addData("Wheel", "rightBack");
        } else if(speedy < NUM_TICKS*4) {
            rightBack.setPower(0);
            rightFront.setPower(POWER);
            leftFront.setPower(0);
            leftBack.setPower(0);
            telemetry.addData("Wheel", "rightFront");
        } else {
            speedy = 0;
        }
    }
}