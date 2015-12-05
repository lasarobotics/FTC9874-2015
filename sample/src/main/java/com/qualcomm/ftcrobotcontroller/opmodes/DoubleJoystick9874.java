package com.qualcomm.ftcrobotcontroller.opmodes;

import com.lasarobotics.library.controller.ButtonState;
import com.lasarobotics.library.controller.Controller;
import com.lasarobotics.library.util.MathUtil;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Will and Russell on 11/7/2015.
 * Features:
 * -- Forward and backwards motion with left joystick y-axis
 * -- Left and right turning while moving or stationary with right joystick x-axis
 * -- Apply speed boost with bottom right trigger
 * -- Apply slowed speed with bottom left trigger
 * -- Toggle smart stopping using y
 */
public class DoubleJoystick9874 extends OpMode {
    Controller one = new Controller(gamepad1);

    //Threshold for stick noise
    public static final double STICK_THRESHOLD = 0.05;
    //Forward/backwards modifiers
    public static final double DAMPENING_FORWARD = 0.5;
    public static final double INCREASED_DAMPENING = 0.8;
    //Turning modifiers
    public static final double TURN_INCREASE = 0.75;
    public static final float MIN_TURN = 0.10f;

    public static final int LEFT_FRONT_MULTIPLIER = 1;
    public static final int LEFT_BACK_MULTIPLIER = 1;
    public static final int RIGHT_FRONT_MULTIPLIER = 1;
    public static final int RIGHT_BACK_MULTIPLIER = 1;

    //Servo variable
    private double servoPower = 0;

    DcMotor leftBack, rightBack, leftFront, rightFront, arm, armEnd;
    Servo servo;

    @Override
    public void init() {
        //Assign variables to motors
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        arm = hardwareMap.dcMotor.get("arm");
        armEnd = hardwareMap.dcMotor.get("armEnd");
        servo = hardwareMap.servo.get("servo");
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        //Read info from controller
        one.update(gamepad1);

        //Servo
        if(one.b == 3.0) {
            if((servoPower += 0.05) > 1) {
                servoPower = 1;
            }
        }
        if(one.x == 3.0) {
            if((servoPower -= 0.05) < -1) {
                servoPower = -1;
            }
        }
        servo.setPosition(servoPower);

        //Arm
        if(one.y == 3.0) {
            arm.setPower(0.1);
        } else if(one.a == 3.0) {
            arm.setPower(-0.1);
        } else {
            arm.setPower(0);
        }

        //ArmEnd
        if(one.dpad_down == 3.0) {
            armEnd.setPower(0.1);
        } else if(one.dpad_up == 3.0) {
            armEnd.setPower(-0.1);
        } else {
            armEnd.setPower(0);
        }

        //Read right stick to determine where to turn, account for controller noise, and scale
        float xVal = (float) MathUtil.deadband(STICK_THRESHOLD, one.right_stick_x * (1 + TURN_INCREASE));
        //Make sure xVal is within acceptable parameters
        xVal = (float)MathUtil.coerce(-1.0, 1.0, (double)xVal);

        //Coefficient for yVal, if right trigger is pressed, apply turbo, if left trigger, apply
        //increased dampening. If none, apply normal dampening.
        double yCoeff = 1-(one.right_trigger == 1.0f ? 0 : (one.left_trigger == 1.0f ? INCREASED_DAMPENING : DAMPENING_FORWARD));
        //Read left stick value to determine motor speed, account for controller noise, and scale
        //down. Then multiply by yCoeff and deadband.
        float yVal = (float)(MathUtil.deadband(STICK_THRESHOLD, one.left_stick_y) * yCoeff);

        //Move robot
        if(xVal > 0 && Math.abs(yVal) > 0) {
            //If moving forward or backward and right
            leftBack.setPower(-yVal * LEFT_BACK_MULTIPLIER);
            leftFront.setPower(-yVal * LEFT_FRONT_MULTIPLIER);
            setRightMotors(xVal, yVal);
        } else if(xVal < 0 && Math.abs(yVal) > 0) {
            //If moving forward or backward and left
            rightBack.setPower(yVal * RIGHT_BACK_MULTIPLIER);
            rightFront.setPower(yVal * RIGHT_FRONT_MULTIPLIER);
            setLeftMotors(xVal, yVal);
        } else if(xVal != 0 && yVal == 0) {
            //If turning stationary
            stationaryTurn(xVal);
        } else if(xVal == 0 && yVal != 0) {
            //If going straight
            leftFront.setPower(-yVal * LEFT_FRONT_MULTIPLIER);
            leftBack.setPower(-yVal * LEFT_BACK_MULTIPLIER);
            rightFront.setPower(yVal * RIGHT_FRONT_MULTIPLIER);
            rightBack.setPower(yVal * RIGHT_BACK_MULTIPLIER);
        } else {
            //Going nowhere
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
    }

    private void setLeftMotors(float xVal, float yVal) {
        float leftPower = Math.abs(yVal) * (1 + xVal);
        leftPower = (leftPower < MIN_TURN && xVal < 0 && Math.abs(yVal) > MIN_TURN) ? MIN_TURN : leftPower;
        if(yVal < 0) {
            leftPower = -leftPower;
        }
        leftBack.setPower(-leftPower * LEFT_BACK_MULTIPLIER);
        leftFront.setPower(-leftPower * LEFT_FRONT_MULTIPLIER);
    }
    private void setRightMotors(float xVal, float yVal) {
        float rightPower = Math.abs(yVal) * (1 - xVal);
        rightPower = (rightPower < MIN_TURN && Math.abs(yVal) > MIN_TURN) ? MIN_TURN : rightPower;
        if(yVal < 0) {
            rightPower = -rightPower;
        }
        rightBack.setPower(rightPower * RIGHT_BACK_MULTIPLIER);
        rightFront.setPower(rightPower * RIGHT_FRONT_MULTIPLIER);
    }
    private void stationaryTurn(float xVal) {
        leftBack.setPower(xVal * LEFT_BACK_MULTIPLIER);
        leftFront.setPower(xVal * LEFT_FRONT_MULTIPLIER);
        rightBack.setPower(xVal * RIGHT_BACK_MULTIPLIER);
        rightFront.setPower(xVal * RIGHT_FRONT_MULTIPLIER);
    }

    @Override
    public void stop() {
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }
}
