package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Russell on 12/5/2015.
 */
public class ServoTest extends OpMode {
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
    }

    double servoValue = -1;
    boolean direction = true;
    @Override
    public void loop() {
        if(direction) {
            servoValue += 0.01;
        } else {
            servoValue -= 0.01;
        }
        if(servoValue < 0) {
            servoValue = 0;
            direction = true;
        } else if(servoValue > 1) {
            servoValue = 1;
            direction = false;
        }
        servo.setPosition(servoValue);
    }
}
