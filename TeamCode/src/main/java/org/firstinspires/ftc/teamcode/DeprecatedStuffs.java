package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static java.lang.Math.abs;

public abstract class DeprecatedStuffs extends LinearOpMode {
    private final int TARGET_WIDTH = 90;
    public List<PixySignature> sigs;
    protected DcMotor leftFront;
    protected DcMotor rightFront;
    protected DcMotor rightRear;
    protected DcMotor leftRear;
    protected SquareRootsVuforia vuforia;
    protected PixyManager pixy;
    protected Servo cameraServo;
    double time = System.currentTimeMillis();
    private boolean _hitGoldBlock = false;
    private boolean goldfound;
    private boolean _readyToHit;

    protected void HitGoldBlock() {

        if (_hitGoldBlock)
            //If it already hit the gold block, the program won't do a single other thing.
            return;
        if (_readyToHit)
            DoTheHit();

        Recognition rec = vuforia.getGoldRecognition();
        if (rec != null) {
            if (rec.getWidth() > 120) {
                // check angle
                if (vuforia.getGoldAngle() < 11) {
                    double scale = 0.1;
                    turn(scale);
                    telemetry.addData("Turning", scale);
                } else {
                    double pow = 0.0;
                    leftRear.setPower(pow);
                    rightRear.setPower(pow);
                    leftFront.setPower(pow);
                    rightFront.setPower(pow);
                    telemetry.addData("Done", null);
                    _readyToHit = true;
                }
            } else if (rec.getWidth() < 120) {
                if (Math.abs(vuforia.getGoldAngle()) > 5) {
                    double scale = 0.1 * -Math.signum(vuforia.getGoldAngle());
                    turn(scale);
                    telemetry.addData("Straightening", scale);
                } else {
                    double pow = 0.3;
                    leftRear.setPower(pow);
                    rightRear.setPower(pow);
                    leftFront.setPower(pow);
                    rightFront.setPower(pow);
                    telemetry.addData("Approaching", rec.getWidth());
                }
            } else {
                telemetry.addData("We have entered the twilight zone", rec.getWidth());
            }
            telemetry.update();
        } else {
            if ((System.currentTimeMillis() - time) > 2000) {
                double pow = -.2;
                leftRear.setPower(pow);
                rightRear.setPower(pow);
                leftFront.setPower(pow);
                rightFront.setPower(pow);
                sleep(500);
            }
            telemetry.addData("I don't see ", "it");
            double pow = 0.0;
            leftRear.setPower(pow);
            rightRear.setPower(pow);
            leftFront.setPower(pow);
            rightFront.setPower(pow);
            telemetry.update();
        }
    }
    private void DoTheHit() {
        sigs = pixy.Run();
        //pixy.run updates the signatures.
        int viewCenter = 160;
        //The center in pixels.
        int viewHeight = 200;
        // How many pixels in total there are along the y-axis.
        PixySignature yellowSig = sigs.get(0);
        //Yellow signature was set to 0, so we called 0.
        int diffY = Math.round(yellowSig.y - viewHeight / 2);
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
                turn(turnScale);
                telemetry.addData("Size: " + sigs.get(0).width + ", " + sigs.get(0).height, null);
                telemetry.update();
            }
            // based on distance, determine how far to move. Either 50 "ticks" or the distance, according
            // to our "distanceToGold" value, whichever is smaller
            RunMotorsToPosition(50, 0.5);
            telemetry.addData("Found gold block", "true");
        }
        RunMotorsToPosition(40, .5);
        cameraServo.setPosition(-90);
        _hitGoldBlock = true;
    }
    boolean findTheGold() {
        if (goldfound) {
            return true;
        }
        double turnspeed = 0.3;
        int waitLength = 1000;
        double angle = vuforia.getGoldAngle();
        if (Math.abs(angle) < 20) {
            telemetry.addData("goldFound", "true");
            telemetry.update();
            goldfound = true;
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            rightFront.setPower(0);
            return true;
        }

        long t = System.currentTimeMillis();
        long end = t + waitLength;

        while (System.currentTimeMillis() < end) { //While a second hasn't passed.
            angle = vuforia.getGoldAngle();
            if (Math.abs(angle) < 20) {
                telemetry.addData("goldFound", "true");
                telemetry.update();
                goldfound = true;
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                rightFront.setPower(0);
                return true;
            }
            leftFront.setPower(turnspeed);
            rightFront.setPower(-turnspeed);
            leftRear.setPower(turnspeed);
            rightRear.setPower(-turnspeed);
        }
        waitLength += 2000;
        end = System.currentTimeMillis() + waitLength;
        while (System.currentTimeMillis() < end) {
            angle = vuforia.getGoldAngle();
            if (Math.abs(angle) < 20) {
                telemetry.addData("goldFound", "true");
                telemetry.update();
                goldfound = true;
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                rightFront.setPower(0);
                return true;
            }
            leftFront.setPower(-turnspeed);
            rightFront.setPower(turnspeed);
            leftRear.setPower(-turnspeed);
            rightRear.setPower(turnspeed);
        }
        telemetry.addData("goldFound", "false");
        telemetry.update();
        return false;

    }
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
    public enum TurnDirection {
        LEFT,
        RIGHT
    }



    // NON DEPRECATED METHODS. USED IN DEPRECATED METHODS


    protected void turn(double turnScale) {
        // Assuming that turnScale isn't 0, it doesn't matter what you put in, since  --x = x.
        double clippedTurnScale = Range.clip(turnScale, -1.0, 1.0);
        rightRear.setPower(-clippedTurnScale);
        rightFront.setPower(-clippedTurnScale);
        leftFront.setPower(clippedTurnScale);
        leftRear.setPower(clippedTurnScale);
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
        while (abs(leftFront.getCurrentPosition()) < abs(targetPos) || abs(rightFront.getCurrentPosition()) < abs(targetPos)
                || abs(leftRear.getCurrentPosition()) < abs(targetPos) || abs(rightRear.getCurrentPosition()) < abs(targetPos)) {
            //This loop continuously tests to see if the individual motors have reached the target position.
            sleep(10);
            if (abs(leftFront.getTargetPosition()) >= abs(targetPos))
                leftFront.setPower(0);
            if (abs(rightFront.getTargetPosition()) >= abs(targetPos))
                rightFront.setPower(0);
            if (abs(leftRear.getTargetPosition()) >= abs(targetPos))
                leftRear.setPower(0);
            if (abs(rightRear.getTargetPosition()) >= abs(targetPos))
                rightRear.setPower(0);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        //At the very end it turns off all the motors.
    }
}
