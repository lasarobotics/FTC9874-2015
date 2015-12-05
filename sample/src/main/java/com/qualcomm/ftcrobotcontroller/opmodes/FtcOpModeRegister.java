/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

/**
 * Register Op Modes
 */
public class FtcOpModeRegister implements OpModeRegister {

    /**
     * The Op Mode Manager will call this method when it wants a list of all
     * available op modes. Add your op mode to the list to enable it.
     *
     * @param manager op mode manager
     */
    public void register(OpModeManager manager) {
        //Custom op modes
        manager.register("Null", NullOp.class);
        manager.register("Mecanum Prototype", MecanumPrototypeTeleop.class);
        //manager.register("MonkeyC Do", MonkeyCDo.class);
        //manager.register("MonkeyC Write", MonkeyCWrite.class);
        //manager.register("MonkeyC 2.0 Do", MonkeyC2Do.class);
        //manager.register("MonkeyC 2.0 Write", MonkeyC2Write.class);
        manager.register("Optical Distance Sensor Tester", DistanceSensorTester.class);
        manager.register("LiftTest", LiftTest.class);
        manager.register("LoggingSample", LoggingSample.class);
        manager.register("OptionsSample", OptionsSample.class);
        manager.register("FTC9874 DriveTrain", DriveTrain9874.class);
        manager.register("Double Joystick 9874", DoubleJoystick9874.class);
        manager.register("Motor Calibration", MotorNameTest.class);
        manager.register("Easy Autonomous 9874", EasyAutonomous9874.class);
        manager.register("Test Opmode 9874", TestOpmode.class);
        manager.register("Button Test Opmode 9874", ButtonTestOpmode.class);
    }
}
