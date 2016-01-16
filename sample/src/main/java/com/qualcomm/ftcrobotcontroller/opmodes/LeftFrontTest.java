package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by S2061383 on 1/12/2016.
 */
public class LeftFrontTest extends OpMode {
    DcMotor leftFront;

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
    }

    boolean increment = true;
    int counter = 0;
    @Override
    public void loop() {
        if(counter < 0 || counter > 1000) {
            increment = !increment;
        }
        if(increment) {
            leftFront.setPower(0.1);
            counter++;
        } else {
            leftFront.setPower(-0.1);
            counter--;
        }
    }
}
