package com.qualcomm.ftcrobotcontroller.opmodes;

import com.lasarobotics.library.controller.Controller;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by S2061383 on 1/12/2016.
 */
public class ControllerTest extends OpMode {
    Controller controller = new Controller(gamepad1);

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        controller.update(gamepad1);
        telemetry.addData("Left stick x", controller.left_stick_x);
        telemetry.addData("A button", controller.a);
    }
}