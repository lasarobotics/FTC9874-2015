package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.lasarobotics.library.nav.EncodedMotor;
import com.lasarobotics.library.nav.MotorInfo;
import com.lasarobotics.library.util.Units;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.BeaconExtension;
import org.opencv.core.Size;

public class TestVisionOpmode extends VisionOpMode {
    BeaconExtension bce = new BeaconExtension();
    private int beaconWaitCounter = 0;
    private int lastBeaconCenterX = 0;
    private int absoluteCenter; //Image width divided by two
    private int moveThreshold; //Allowed deviation of beacon from image center
    private int moveForwardTimer = 0;
    private int moveBackwardTimer = 0;
    private boolean initialMoveForward = true;
    private boolean isInitialMoveForwardPartTwo = true;
    private boolean stop = false;
    private static final int BEACON_WAIT_TIME = 200; //Time to wait before rotation if unable to find beacon
    private static final double ROTATE_MOTOR_POWER = 0.3; //Motor power when rotating
    private double DRIVE_MOTOR_POWER = 0.3; //Motor power when driving
    private static final double UNCERTAIN_MOTOR_POWER = 0; //Motor power when unsure what to do
    DcMotor leftBack, rightBack, leftFront, rightFront; //Motors
    EncodedMotor leftBackEncoded, rightBackEncoded, leftFrontEncoded, rightFrontEncoded; //Encoders

    @Override
    public void init() {
        super.init();

        //START VISION JUNK
        //Set the camera used for detection
        this.setCamera(Cameras.PRIMARY);
        //Set the frame size
        //Larger = sometimes more accurate, but also much slower
        //For Testable OpModes, this might make the image appear small - it might be best not to use this
        this.setFrameSize(new Size(900, 900));

        //Enable extensions. Use what you need.
        enableExtension(Extensions.BEACON);     //Beacon detection
        enableExtension(Extensions.ROTATION);   //Automatic screen rotation correction

        //UNCOMMENT THIS IF you're using a SECONDARY (facing toward screen) camera
        //or when you rotate the phone, sometimes the colors swap
        //rotation.setRotationInversion(true);

        //You can do this for certain phones which switch red and blue
        //It will rotate the display and detection by 180 degrees, making it upright
        //rotation.setUnbiasedOrientation(ScreenOrientation.LANDSCAPE_WEST);

        //Set the beacon analysis method
        //Try them all and see what works!
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
        //END VISION JUNK

        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
    }

    public void encoderEnable() {
        rightBackEncoded.enableEncoder();
        rightFrontEncoded.enableEncoder();
        leftBackEncoded.enableEncoder();
        leftFrontEncoded.enableEncoder();
    }

    public void encoderDisable() {
        rightBackEncoded.disableEncoder();
        rightFrontEncoded.disableEncoder();
        leftBackEncoded.disableEncoder();
        leftFrontEncoded.disableEncoder();
    }

    public void encoderInit() {
        MotorInfo info = new MotorInfo(3, Units.Distance.INCHES);
        leftBackEncoded = new EncodedMotor(leftBack, info);
        rightBackEncoded = new EncodedMotor(rightBack, info);
        leftFrontEncoded = new EncodedMotor(leftFront, info);
        rightFrontEncoded = new EncodedMotor(rightFront, info);
    }

    public void encoderReset() {
        rightBackEncoded.reset();
        rightFrontEncoded.reset();
        leftBackEncoded.reset();
        leftFrontEncoded.reset();
    }

    private int initialEncoderReverse = 0;
    public void encoderLoop() {
        leftBackEncoded.update();
        rightBackEncoded.update();
        leftFrontEncoded.update();
        rightFrontEncoded.update();
        if(initialEncoderReverse == 0) {
            leftBackEncoded.moveDistance(-1, Units.Distance.METERS);
            leftFrontEncoded.moveDistance(-1, Units.Distance.METERS);
            rightBackEncoded.moveDistance(1, Units.Distance.METERS);
            rightFrontEncoded.moveDistance(1, Units.Distance.METERS);
            initialEncoderReverse = 1;
        } else if(initialEncoderReverse == 1) {
            if (leftBackEncoded.getCurrentPosition(Units.Distance.METERS) <= -0.95) {
                initialEncoderReverse = 2;
            }
        } else if(initialEncoderReverse == 2) {
            rightBackEncoded.reset();
            rightFrontEncoded.reset();
            initialEncoderReverse = 3;
        } else if(initialEncoderReverse == 3) {
            if(rightFrontEncoded.hasEncoderReset()) {
                rightBackEncoded.moveDistance(0.2, Units.Distance.METERS);
                rightFrontEncoded.moveDistance(0.2, Units.Distance.METERS);
                initialEncoderReverse = 4;
            }
        } else if(initialEncoderReverse == 4) {
            if(rightBackEncoded.getCurrentPosition(Units.Distance.METERS) >= 0.15) {
                stop();
            }
        }
    }
    int count2 = 0;
    int count3 = 0;
    int count4 = 0;
    int count5 = 0;
    public void otherLoop() {
        leftBackEncoded.update();
        rightBackEncoded.update();
        leftFrontEncoded.update();
        rightFrontEncoded.update();
        if(initialEncoderReverse == 0) {
            leftBack.setPower(-1);
            leftFront.setPower(-1);
            rightBack.setPower(1);
            rightFront.setPower(1);
            initialEncoderReverse = 1;
        } else if(initialEncoderReverse == 1) {
            if (++count2 > 300) {
                initialEncoderReverse = 2;
            }
        } else if(initialEncoderReverse == 2) {
            //rightBackEncoded.reset();
            //rightFrontEncoded.reset();
            if(++count3 > 100) {
                initialEncoderReverse = 3;
            }
        } else if(initialEncoderReverse == 3) {
            rightBack.setPower(0.3);
            rightFront.setPower(0.3);
            leftBack.setPower(0.3);
            rightBack.setPower(0.3);
            initialEncoderReverse = 4;
            //if(rightFrontEncoded.hasEncoderReset()) {
            //    rightBackEncoded.moveDistance(0.2, Units.Distance.METERS);
            //    rightFrontEncoded.moveDistance(0.2, Units.Distance.METERS);
            //    initialEncoderReverse = 4;
            //}
        } else if(initialEncoderReverse == 4) {
            if(++count4 > 200) {
                initialEncoderReverse = 5;
            }
        } else if(initialEncoderReverse == 5) {
            DRIVE_MOTOR_POWER = 0.8;
            driveForward();
            initialEncoderReverse = 6;
        } else if(initialEncoderReverse == 6) {
            if(++count5 < 600) {
                unsureMotors();
                stop();
            }
        }
    }

    int count1 = 0;
    boolean initServo = false;
    @Override
    public void loop() {
        super.loop();

        if(!initServo) {
            Servo arm = hardwareMap.servo.get("arm");
            if(arm != null) {
                arm.setPosition(0);
            }
        }

        if(!isInitialMoveForwardPartTwo) {
            telemetry.addData("Initial move forward2", "true");
            telemetry.addData("Pos", String.valueOf(rightBackEncoded.getCurrentPosition()));
            if(count1 > 300) {
                isInitialMoveForwardPartTwo = true;
                //encoderDisable();
            }
            return;
        }
        if(initialMoveForward) {
            telemetry.addData("Initial move forward", "true");
            //encoderInit();
            //encoderEnable();
            //encoderReset();
            //leftBackEncoded.moveDistance(-1, Units.Distance.METERS);
            //leftFrontEncoded.moveDistance(-1, Units.Distance.METERS);
            //rightBackEncoded.moveDistance(1, Units.Distance.METERS);
            //rightFrontEncoded.moveDistance(1, Units.Distance.METERS);
            isInitialMoveForwardPartTwo = false;
            initialMoveForward = false;
            return;
        }

        //Vision telemetry
        telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
        telemetry.addData("Beacon Location (Center)", beacon.getAnalysis().getLocationString());
        telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
        telemetry.addData("Rotation Compensation", rotation.getRotationCompensationAngle());
        telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
        telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);

        if(stop) {
            otherLoop();
            return;
        }
        if(moveForwardTimer > 0) {
            moveForwardTimer--;
            if(moveForwardTimer == 0) {
                moveBackwardTimer = 500;
            }
            driveForward();
            try {
                Thread.sleep(1);
            } catch(Exception e) {}
        }
        if(moveBackwardTimer > 0) {
            //encoderReset();
            //encoderEnable();
            Servo arm = hardwareMap.servo.get("arm");
            if(arm != null) {
                arm.setPosition(1);
            }
            stop = true;
            return;
            /*moveBackwardTimer--;
            if(moveBackwardTimer == 0) {
                encoderInit();
                stop = true;
            }
            driveBackward();
            try {
                Thread.sleep(1);
            } catch(Exception e) {}*/
        }
        bce.loop(this);
        absoluteCenter = height/2;
        moveThreshold = height/19;

        Beacon.BeaconAnalysis analysis = bce.getAnalysis();
        telemetry.addData("FOUND?", analysis.getStateRight());
        int beaconCenterX = (int)(analysis.getTopLeft().x + analysis.getBottomRight().x)/2;
        if(analysis.getStateRight() != Beacon.BeaconColor.UNKNOWN) {
            if(!withinThreshold(beaconCenterX, lastBeaconCenterX, moveThreshold)) {
                //Last known location of beacon is drastically different from current location
                if(beaconWaitCounter++ < BEACON_WAIT_TIME) {
                    unsureMotors();
                    return;
                }
            }
            if(Math.abs(analysis.getTopLeft().x - analysis.getBottomRight().x) < 200) { //close enough
                moveForwardTimer = 500;
            }
            shouldMove(beaconCenterX);
            beaconWaitCounter = 0;
            lastBeaconCenterX = beaconCenterX;
        } else {
            findBeacon(beaconCenterX);
        }
    }

    public void findBeacon(int beaconCenterX) {
        //Have we waited long enough?
        if(beaconWaitCounter < BEACON_WAIT_TIME) {
            beaconWaitCounter++;
            unsureMotors();
            return; //No we haven't
        }

        //Yes we have
        rotateLeft();
    }

    public void rotateLeft() {
        rightFront.setPower(-ROTATE_MOTOR_POWER);
        rightBack.setPower(-ROTATE_MOTOR_POWER);
        leftFront.setPower(-ROTATE_MOTOR_POWER);
        leftBack.setPower(-ROTATE_MOTOR_POWER);
    }

    public void rotateRight() {
        leftFront.setPower(ROTATE_MOTOR_POWER);
        leftBack.setPower(ROTATE_MOTOR_POWER);
        leftFront.setPower(ROTATE_MOTOR_POWER);
        leftBack.setPower(ROTATE_MOTOR_POWER);
    }

    public void driveForward() {
        rightFront.setPower(-DRIVE_MOTOR_POWER);
        rightBack.setPower(-DRIVE_MOTOR_POWER);
        leftFront.setPower(DRIVE_MOTOR_POWER);
        leftBack.setPower(DRIVE_MOTOR_POWER);
    }

    public void driveBackward() {
        rightFront.setPower(DRIVE_MOTOR_POWER);
        rightBack.setPower(DRIVE_MOTOR_POWER);
        leftFront.setPower(-DRIVE_MOTOR_POWER);
        leftBack.setPower(-DRIVE_MOTOR_POWER);
    }

    public void unsureMotors() {
        rightFront.setPower(-UNCERTAIN_MOTOR_POWER);
        rightBack.setPower(-UNCERTAIN_MOTOR_POWER);
        leftFront.setPower(UNCERTAIN_MOTOR_POWER);
        leftBack.setPower(UNCERTAIN_MOTOR_POWER);
    }

    public void shouldMove(int beaconCenterX) {
        if(withinThreshold(beaconCenterX, absoluteCenter, moveThreshold)) {
            driveForward();
        } else {
            if(beaconCenterX < absoluteCenter) {
                rotateLeft();
            } else {
                rotateRight();
            }
        }
    }

    public static boolean withinThreshold(int item1, int item2, int threshold) { //TODO move to helper method in some other class
        int distanceFrom = Math.abs(item1 - item2);
        return distanceFrom <= threshold;
    }

    @Override
    public void stop() {
        super.stop();
        bce.stop(this);
    }
}