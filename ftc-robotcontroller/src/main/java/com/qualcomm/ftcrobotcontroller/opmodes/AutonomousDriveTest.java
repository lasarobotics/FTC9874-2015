package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Russell on 1/20/2016.
 */
public class AutonomousDriveTest extends OpMode {
    public static final int NUM_TICKS = 200;
    public static final double POWER = 0.2;
    //Wheel change constants
    public static final int LEFT_MODIFIER = -1;
    public static final int RIGHT_MODIFIER = 1;

    DcMotor leftBack, rightBack, leftFront, rightFront;

    @Override
    public void init() {
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
    }

    int speedy = 0;
    boolean movingForward = true;
    @Override
    public void loop() {
        if(++speedy > 200) {
            speedy = 0;
            movingForward = !movingForward;
            rightBack.setPower((movingForward ? 0.2 : -0.2) * RIGHT_MODIFIER);
            rightFront.setPower((movingForward ? 0.2 : -0.2) * RIGHT_MODIFIER);
            leftBack.setPower((movingForward ? -0.2 : 0.2) * LEFT_MODIFIER);
            leftFront.setPower((movingForward ? -0.2 : 0.2) * LEFT_MODIFIER);
        }
    }
}
