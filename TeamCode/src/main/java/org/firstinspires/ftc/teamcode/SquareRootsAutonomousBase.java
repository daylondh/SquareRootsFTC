package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static java.lang.Math.abs;

public abstract class SquareRootsAutonomousBase extends LinearOpMode {

    protected boolean _landingDone;
    protected DcMotor leftFront;
    protected DcMotor rightFront;
    protected DcMotor rightRear;
    protected DcMotor leftRear;
    protected DcMotor lift;
    protected DcMotor arm;
    protected Gyroscope imu;
    protected SquareRootsVuforia vuforia;
    protected PixyManager pixy;
    protected boolean _clearedLander;
    private boolean _hitGoldBlock = false;
    protected Servo cameraServo;
    protected Servo leftTooth;
    protected Servo rightTooth;
    protected Servo wrist;
    protected DcMotor shoulder;
    private final int TARGET_WIDTH = 90;
    public List<PixySignature> sigs;
    private boolean goldfound;


    protected void Init() {
        //Maps motors and servos to their corresponding position on the hardwaremap,
        // updates the telemetry, sets up vuforia, sets the motors to run with encoder.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        _landingDone = false;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        lift = hardwareMap.get(DcMotor.class, "lift");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        pixy = new PixyManager(hardwareMap.i2cDeviceSynch.get("pixy"));
        leftTooth = hardwareMap.get(Servo.class, "leftTooth");
        rightTooth = hardwareMap.get(Servo.class, "rightTooth");
        wrist = hardwareMap.get(Servo.class, "wrist");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforia = new SquareRootsVuforia(cameraMonitorViewId);


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void DoLanding() {
        //Does the landing.
        // TODO: 11/13/2018 Add a failsafe. If the drive team used b to go up, this will NOT WORK!!!
        if (_landingDone)
            return;
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(7300);
        lift.setPower(1);
        while (lift.isBusy()) {
            sleep(10);
        }
        lift.setPower(0);
        turn(-20);

        double strafe = -1.0;
        double leftFrontPower = -strafe;
        double rightFrontPower = strafe;
        double leftRearPower = strafe;
        double rightRearPower = -strafe;



        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
        sleep(1100);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sleep(10);

        _landingDone = true;
    }

    protected void ClearLander() {
        //Well, yeah. It clears the lander. That's about it.
        if (_clearedLander)
            return;
        cameraServo.setPosition(2.0);
        int targetPos = 150;
        double targetSpeed = 0.8;
        RunMotorsToPosition(targetPos, targetSpeed);

        _clearedLander = true;
    }

    public void RunMotorsToPosition(int targetPos, double targetSpeed) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Sets all the motors to run with encoders, so that we can measure the distance the motor has gone.
        leftFront.setPower(targetSpeed);
        rightFront.setPower(targetSpeed);
        leftRear.setPower(targetSpeed);
        rightRear.setPower(targetSpeed);
        // Run motors at target speed.
        while (leftFront.getCurrentPosition() < targetPos || rightFront.getCurrentPosition() < targetPos
                || leftRear.getCurrentPosition() < targetPos || rightRear.getCurrentPosition() < targetPos) {
           //This loop continuously tests to see if the individual motors have reached the target position.
            sleep(10);
            if (leftFront.getTargetPosition() >= targetPos)
                leftFront.setPower(0);
            if (rightFront.getTargetPosition() >= targetPos)
                rightFront.setPower(0);
            if (leftRear.getTargetPosition() >= targetPos)
                leftRear.setPower(0);
            if (rightRear.getTargetPosition() >= targetPos)
                rightRear.setPower(0);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        //At the very end it turns off all the motors.
    }

    protected void HitGoldBlock() {

        if (_hitGoldBlock)
            //If it already hit the gold block, the program won't do a single other thing.
            return;

        sigs = pixy.Run();
        //pixy.run updates the signatures.
        int viewCenter = 160;
        //The center in pixels.
        int viewHeight = 200;
        // How many pixels in total there are along the y-axis.
        PixySignature yellowSig = sigs.get(0);
        //Yellow signature was set to 0, so we called 0.
        int diffY = yellowSig.y - viewHeight / 2;
        //Explained inside the while loops.

        while (yellowSig.width < TARGET_WIDTH) {
            //Basically tries to estimate the distance using the ratio of gold pixels to overall pixels.
            sigs = pixy.Run();
            //Update sigs variable.
            yellowSig = sigs.get(0);
            //In PixyMon, we made the yellow signature be sig 1. However, arrays start at 0, so in the code it's
            // called as 0.
            diffY = yellowSig.y - viewHeight / 2;
            //the y axis of the gold block minus the x axis of the camera's aspect ratio... /2
            while (abs(diffY) > 1) {
                //Since yellowsig.y will always be less than the y axis of the aspect ratio until we are close enough to the block,
                // the reading should be negative.
                // Remember, the pixy is turned sideways, so x = y.
                sigs = pixy.Run();
                //Do the exact same stuff as the previous loop.
                yellowSig = sigs.get(0);
                diffY = yellowSig.y - viewHeight / 2;
                double turnScale = 1.0 * diffY / (viewHeight / 2);
                if (turnScale > .75) {

                }
                turn(turnScale);
                telemetry.addData("Size: " + sigs.get(0).width + ", " + sigs.get(0).height, null);
                telemetry.update();
            }
            double theta = vuforia.getGoldAngle(AngleUnit.RADIANS);
            double distanceToGold = (4.75 / Math.tan(theta)) - 6;
            // based on distance, determine how far to move. Either 50 "ticks" or the distance, according
            // to our "distanceToGold" value, whichever is smaller
            RunMotorsToPosition(50, 0.5);
            telemetry.addData("Found gold block", "true");
        }
        RunMotorsToPosition(40, .5);
        cameraServo.setPosition(-90);
        _hitGoldBlock = true;
    }

    boolean findTheGold() throws InterruptedException {
        if (goldfound) {
            //If the gold is already in the pixy's FOV, this will return true.
            return true;
        }
        long t = System.currentTimeMillis();
        long end = t + 2000;
        //Gives a certain amount of time to run the while loop.
        //We decided on 2 seconds, but we can change that later.
        while (System.currentTimeMillis() < end) {
            sigs = pixy.Run();
            //Updates the pixycam
            if (sigs.get(0).width > 60) {
                // If there are more than 60 pixels of gold, it'll know that it's the mineral we're after,
                // and not one inside the crater or something off in the distance.
                telemetry.addData("goldFound", "true");
                telemetry.update();
                goldfound = true;
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                rightFront.setPower(0);
                //This stops the robot, so it won't drive in circles, and returns true, breaking the loop.
                return true;
            }
            //Turn the robot right.
            leftFront.setPower(.5);
            rightFront.setPower(-.5);
            leftRear.setPower(.5);
            rightRear.setPower(-.5);
        }
        end = System.currentTimeMillis() + 2000;
        //Reset the timer.
        while (System.currentTimeMillis() < end) {
            //Do the exact same stuff as the previous while loop, except it turns the opposite way.
            sigs = pixy.Run();
            if (sigs.get(0).width > 60) {
                telemetry.addData("Found gold block.", null);
                telemetry.update();
                goldfound = true;
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                rightFront.setPower(0);
                return true;
            }
            leftFront.setPower(-.5);
            rightFront.setPower(.5);
            leftRear.setPower(-.5);
            rightRear.setPower(.5);
        }
        //If neither scan found the gold, we assume that we just aren't close enough and drive forward 90 degrees (72 ticks)
        RunMotorsToPosition(72, .5);
        return false;

    }
    @Deprecated //This function was used for tensorflow when we were trying to use it
    private double calculateDistance(Recognition recognition) {
        final int measure = 6;
        //How far the robot moves forward.
        double theta1 = recognition.estimateAngleToObject(AngleUnit.RADIANS);
        //Theta1 is our first angle.
        RunMotorsToPosition(180, .75);
        //Running motors 180 degrees brings us forward half a foot.
        double theta2 = recognition.estimateAngleToObject(AngleUnit.RADIANS);
        //Theta2 is our second angle.
        double distance = (measure * Math.tan(theta1) * Math.tan(theta2)) / Math.tan(theta2) - Math.tan(theta1);
        // Multiplies 6 by the tangent of theta1, and multiplies that by the tangent of theta2.
        // It takes that value, and divides it by the tangent of theta2 minus the tangent of theta1.
        return distance;
    }

    protected void turn(double turnScale) {
        // Assuming that turnScale isn't 0, it doesn't matter what you put in, since  --x = x.
        double clippedTurnScale = Range.clip(turnScale, -1.0, 1.0);
        rightRear.setPower(-clippedTurnScale);
        rightFront.setPower(-clippedTurnScale);
        leftFront.setPower(clippedTurnScale);
        leftRear.setPower(clippedTurnScale);
    }
}
