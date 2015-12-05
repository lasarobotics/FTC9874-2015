package com.qualcomm.ftcrobotcontroller.opmodes;

import com.lasarobotics.library.controller.Controller;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Russell on 12/5/2015.
 */
public class ButtonTestOpmode extends OpMode {
    Controller one = new Controller(gamepad1);

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        one.update(gamepad1);
        telemetry.addData("A", one.a);
        telemetry.addData("B", one.b);
        telemetry.addData("X", one.x);
        telemetry.addData("Y", one.y);
    }
}
