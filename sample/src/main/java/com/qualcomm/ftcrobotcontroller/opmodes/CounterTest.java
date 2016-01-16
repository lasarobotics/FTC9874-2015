package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by S2061383 on 1/12/2016.
 */
public class CounterTest extends OpMode {
    @Override
    public void init() {

    }

    int count = 0;
    @Override
    public void loop() {
        telemetry.addData("Controller", count++);
    }
}
