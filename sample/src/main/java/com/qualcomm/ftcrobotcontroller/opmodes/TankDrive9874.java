package com.qualcomm.ftcrobotcontroller.opmodes;

import com.lasarobotics.library.controller.Controller;
import com.lasarobotics.library.drive.Tank;
import com.lasarobotics.library.util.MathUtil;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Russell on 11/15/2015.
 */
public class TankDrive9874 extends OpMode {

    Controller one = new Controller(gamepad1);

    //Constants
    public static final double STICK_THRESHOLD = 0.05;
    public static final double DAMPENING = 0.5;
    public static final double INCREASED_DAMPENING = 0.8;

    public static final int LEFT_MULTIPLIER = -1;
    public static final int RIGHT_MULTIPLIER = 1;

    //Motors
    DcMotor leftBack, rightBack, leftFront, rightFront;

    @Override
    public void init() {
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        servo = hardwareMap.servo.get("servo");
    }

    @Override
    public void loop() {
        //Read values from controller
        one.update(gamepad1);

        //Determine how to adjust speed of robot
        //If normal, robot speed adjusted by DAMPENING
        //If right trigger, remove dampening.
        //If left trigger, go even slower.
        double adjustedDampening = DAMPENING;
        if(one.right_trigger == 1.0f) {
            adjustedDampening = 0;
        } else if(one.left_trigger == 1.0f) {
            adjustedDampening = INCREASED_DAMPENING;
        }

        //Adjust for slight inconsistencies and apply changes from above
        float leftVal = (float) MathUtil.deadband(STICK_THRESHOLD, one.left_stick_y * (1 - adjustedDampening));
        float rightVal = (float) MathUtil.deadband(STICK_THRESHOLD, one.right_stick_y * (1 - adjustedDampening));
        leftVal *= LEFT_MULTIPLIER;
        rightVal *= RIGHT_MULTIPLIER;

        //Utilize Tank class to move robot
        Tank.motor4(leftFront, rightFront, leftBack, rightBack, leftVal, rightVal);
    }
}
