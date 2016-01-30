package com.qualcomm.ftcrobotcontroller.opmodes;

import com.lasarobotics.library.nav.EncodedMotor;
import com.lasarobotics.library.nav.MotorInfo;
import com.lasarobotics.library.util.Units;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Russell on 1/30/2016.
 */
public class EncoderTest extends OpMode {
    DcMotor leftBack, rightBack, leftFront, rightFront; //Motors
    EncodedMotor leftBackEncoded, rightBackEncoded, leftFrontEncoded, rightFrontEncoded; //Encoders

    @Override
    public void init() {
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");

        rightBack.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightBack.setTargetPosition(30);
        rightBack.setPower(1);
    }

    //boolean set
    @Override
    public void loop() {
        telemetry.addData("Pos", String.valueOf(rightBack.getCurrentPosition()));
    }
}
