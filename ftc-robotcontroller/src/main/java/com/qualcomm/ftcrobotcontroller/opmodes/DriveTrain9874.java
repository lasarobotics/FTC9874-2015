package com.qualcomm.ftcrobotcontroller.opmodes;

import com.lasarobotics.library.controller.Controller;
import com.lasarobotics.library.util.MathUtil;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Will on 10/22/2015.
 */
public class DriveTrain9874 extends OpMode {

    Controller one = new Controller(gamepad1);

    public static final double STICK_THRESHOLD = 0.05;
    public static final double DAMPENING_FORWARD = 0.5;
    public static final double TURN_INCREASE = 0.75;
    public static final float MIN_TURN = 0.10f;

    DcMotor leftBack, rightBack, leftFront, rightFront;

    @Override
    public void init() {
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        one.update(gamepad1);
        float xVal = (float)MathUtil.deadband(STICK_THRESHOLD, one.left_stick_x*(1+TURN_INCREASE));
        xVal = (float)MathUtil.coerce(-1.0, 1.0, (double)xVal);
        float yVal = (float)(MathUtil.deadband(STICK_THRESHOLD, one.left_stick_y)*(1-DAMPENING_FORWARD));
        if(xVal > 0) {
            leftBack.setPower(-yVal);
            leftFront.setPower(-yVal);
            float rightPower = yVal * (1 - xVal);
            rightPower = (rightPower < MIN_TURN && yVal > MIN_TURN) ? MIN_TURN : rightPower;
            rightBack.setPower(rightPower);
            rightFront.setPower(rightPower);
        } else {
            rightBack.setPower(yVal);
            rightFront.setPower(yVal);
            float rightPower = yVal * (1 + xVal);
            rightPower = (rightPower < MIN_TURN && xVal < 0 && yVal > MIN_TURN) ? MIN_TURN : rightPower;
            leftBack.setPower(-rightPower);
            leftFront.setPower(-rightPower);
        }
    }

    @Override
    public void stop() {
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }
}
