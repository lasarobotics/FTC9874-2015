package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Russell on 12/5/2015.
 */
public class EasyAutonomous9874 extends OpMode {
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
            leftBack.setPower(0.1);
            leftFront.setPower(0.1);
            rightBack.setPower(0.1);
            rightFront.setPower(0.1);
        } else {
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        }
    }

    @Override
    public void stop() {
        super.stop();
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
}
