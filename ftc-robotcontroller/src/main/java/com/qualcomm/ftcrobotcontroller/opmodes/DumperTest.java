package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Russell on 1/20/2016.
 */
public class DumperTest extends OpMode {
    Servo dump;

    @Override
    public void init() {
        dump = hardwareMap.servo.get("dump");
    }

    double servoLoc = 0.0d;
    int timer = 0;
    boolean dir = true;
    @Override
    public void loop() {
        timer = 0;
        if(dir) servoLoc += 0.005d;
        else servoLoc -= 0.005d;
        if(servoLoc >= 1) {
            servoLoc = 0.98d;
            dir = false;
        } else if(servoLoc <= 0) {
            servoLoc = 0.02d;
            dir = true;
        }
        dump.setPosition(servoLoc);
    }
}
