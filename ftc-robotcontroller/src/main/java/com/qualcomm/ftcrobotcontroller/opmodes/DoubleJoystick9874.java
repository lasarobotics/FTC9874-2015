package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;
import android.widget.Button;

import com.lasarobotics.library.controller.ButtonState;
import com.lasarobotics.library.controller.Controller;
import com.lasarobotics.library.util.MathUtil;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    //Smart stop constants
    public static final double SMARTSTOP_INCREMENT = 0.1;
    //Drive direction
    boolean forwardDrive = false;
    boolean wasBPressed = false;

    //Smart stop variables
    private boolean smartStop = false;
    private float previousSpeed = 0;

    DcMotor leftBack, rightBack, leftFront, rightFront;

    @Override
    public void init() {
        //Assign variables to motors
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
        //Read info from controller
        one.update(gamepad1);

        //Check for smart stop toggle
        if(one.y == ButtonState.PRESSED)
            smartStop = !smartStop;

        //Drive direction swapper
        /*if(wasBPressed) {
            if(one.b != ButtonState.PRESSED) {
                //When b is released
                forwardDrive = !forwardDrive;
                wasBPressed = false;
            }
        } else if(one.b == ButtonState.PRESSED) {
            //When b is first pressed
            wasBPressed = true;
        }*/

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

        //If smart stop is on apply smart scoring algorithm
        if(smartStop)
            if(previousSpeed - yVal > SMARTSTOP_INCREMENT)
                yVal = (float)(previousSpeed - SMARTSTOP_INCREMENT);

        //Move robot
        if(xVal > 0 && Math.abs(yVal) > 0) {
            //If moving forward or backward and right
            leftBack.setPower(calculateAdjustedValue(-yVal));
            leftFront.setPower(calculateAdjustedValue(-yVal));
            setRightMotors(xVal, yVal);
        } else if(xVal < 0 && Math.abs(yVal) > 0) {
            //If moving forward or backward and left
            rightBack.setPower(calculateAdjustedValue(yVal));
            rightFront.setPower(calculateAdjustedValue(yVal));
            setLeftMotors(xVal, yVal);
        } else if(xVal != 0 && yVal == 0) {
            //If turning stationary
            stationaryTurn(xVal);
        } else if(xVal == 0 && yVal != 0) {
            //If going straight
            leftFront.setPower(calculateAdjustedValue(-yVal));
            leftBack.setPower(calculateAdjustedValue(-yVal));
            rightFront.setPower(calculateAdjustedValue(yVal));
            rightBack.setPower(calculateAdjustedValue(yVal));
        } else {
            //Going nowhere
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }

        //Set previous speed to this loops yVal for use in smart stopping
        previousSpeed = yVal;
    }

    private void setLeftMotors(float xVal, float yVal) {
        float leftPower = Math.abs(yVal) * (1 + xVal);
        leftPower = (leftPower < MIN_TURN && xVal < 0 && Math.abs(yVal) > MIN_TURN) ? MIN_TURN : leftPower;
        if(yVal < 0) {
            leftPower = -leftPower;
        }
        leftBack.setPower(calculateAdjustedValue(-leftPower));
        leftFront.setPower(calculateAdjustedValue(-leftPower));
    }
    private void setRightMotors(float xVal, float yVal) {
        float rightPower = Math.abs(yVal) * (1 - xVal);
        rightPower = (rightPower < MIN_TURN && Math.abs(yVal) > MIN_TURN) ? MIN_TURN : rightPower;
        if(yVal < 0) {
            rightPower = -rightPower;
        }
        rightBack.setPower(calculateAdjustedValue(rightPower));
        rightFront.setPower(calculateAdjustedValue(rightPower));
    }
    private void stationaryTurn(float xVal) {
        leftBack.setPower(calculateAdjustedValue(xVal));
        leftFront.setPower(calculateAdjustedValue(xVal));
        rightBack.setPower(calculateAdjustedValue(xVal));
        rightFront.setPower(calculateAdjustedValue(xVal));
    }

    private double calculateAdjustedValue(double val) {
        return (forwardDrive ? val : val * -1);
    }

    @Override
    public void stop() {
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }
}