package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.SquareRootsAutonomousBase.Axis.Z;

public abstract class SquareRootsAutonomousBase extends LinearOpMode {

    protected boolean _landingDone;
    protected DcMotor leftRear;
    protected DcMotor rightRear;
    protected DcMotor rightFront;
    protected DcMotor leftFront;
    protected DcMotor lift;
    protected Servo arm;
    protected BNO055IMU imu;
    protected SquareRootsVuforia vuforia;
    protected boolean _clearedLander;
    protected Servo duckarm;
    protected GoToPosition pos;
    LanderSide side;
    private boolean _foundGold;
    private boolean _squaredOnGold = false;
    private boolean override;
    private boolean _alignedOnVumark;

    //Constructor requires subclasses to be either gold or silver.
    public SquareRootsAutonomousBase(LanderSide side) {
        this.side = side;
    }

    protected void Init() {
        //Maps motors and servos to their corresponding position on the hardwaremap,
        // updates the telemetry, sets up vuforia, sets the motors to run with encoder.

        telemetry.addData("Initializing", "true");
        telemetry.update();
        _landingDone = false;

        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        lift = hardwareMap.get(DcMotor.class, "lift");
        arm = hardwareMap.get(Servo.class, "armServo");
        duckarm = hardwareMap.get(Servo.class, "duckArm");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforia = new SquareRootsVuforia(cameraMonitorViewId);
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforia.InitTfod(tfodMonitorViewId);


        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pos = new GoToPosition(vuforia, leftRear, rightRear, leftFront, rightFront, imu, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    protected void DoLanding() {
        if(isStopRequested())
            return;
        if (_landingDone)
            return;
        telemetry.addData("Landing", "");
        telemetry.update();
        arm.setPosition(0.5);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(7400);
        lift.setPower(1);
        while (lift.isBusy() && !isStopRequested()) {
            sleep(10);
        }
        if(isStopRequested())
            return;
        lift.setPower(0);

        double strafe = -1.0;
        double leftFrontPower = -strafe;
        double rightFrontPower = strafe;
        double leftRearPower = strafe;
        double rightRearPower = -strafe;
        arm.setPosition(1);

        leftRear.setPower(leftFrontPower);
        rightRear.setPower(rightFrontPower);
        leftFront.setPower(leftRearPower);
        rightFront.setPower(rightRearPower);
        sleep(900);
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        _landingDone = true;
    }

    protected void ClearLander() {
        if(isStopRequested())
            return;
        if (_clearedLander)
            return;
        telemetry.addData("Clearing Lander", "");
        telemetry.update();
        int targetPos = 10;
        double targetSpeed = 0.8;
        turn(-1.0);
        sleep(500);
        RunMotorsToPosition(targetPos, targetSpeed);

        _clearedLander = true;
    }

    public void RunMotorsToPosition(int targetPos, double targetSpeed) {
        if(isStopRequested())
            return;
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Sets all the motors to run with encoders, so that we can measure the distance the motor has gone.
        leftRear.setPower(targetSpeed);
        rightRear.setPower(targetSpeed);
        leftFront.setPower(targetSpeed);
        rightFront.setPower(targetSpeed);
        // Run motors at target speed.
        while (abs(leftRear.getCurrentPosition()) < abs(targetPos) || abs(rightRear.getCurrentPosition()) < abs(targetPos)
                || abs(leftFront.getCurrentPosition()) < abs(targetPos) || abs(rightFront.getCurrentPosition()) < abs(targetPos) && !isStopRequested()) {
            //This loop continuously tests to see if the individual motors have reached the target position.
            sleep(10);
            if (abs(leftRear.getTargetPosition()) >= abs(targetPos))
                leftRear.setPower(0);
            if (abs(rightRear.getTargetPosition()) >= abs(targetPos))
                rightRear.setPower(0);
            if (abs(leftFront.getTargetPosition()) >= abs(targetPos))
                leftFront.setPower(0);
            if (abs(rightFront.getTargetPosition()) >= abs(targetPos))
                rightFront.setPower(0);
        }
        if(isStopRequested())
            return;
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    protected void turn(double turnScale) {
        if(isStopRequested())
            return;
        double clippedTurnScale = Range.clip(turnScale, -1.0, 1.0);
        rightFront.setPower(-clippedTurnScale);
        rightRear.setPower(-clippedTurnScale);
        leftRear.setPower(clippedTurnScale);
        leftFront.setPower(clippedTurnScale);
    }

    protected void strafeToGetGold() {

        if (_foundGold) {
            return;
        }
        telemetry.addData("Strafe To Get Gold", "");
        telemetry.update();
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        long time = System.currentTimeMillis();
        while (!vuforia.seesGold(930) && !isStopRequested()) {
            //telemetry.addData("TensorFlow Data: ", vuforia.getGoldRecognition());
            //telemetry.update();
            // Wait 8 seconds, if it hasn't seen the gold, it gives up.
            if (System.currentTimeMillis() - 7000 > time) {
                //telemetry.addData("Giving up", null);
                //telemetry.update();
                override = true;
                smackAndRun(100, MineralSquare.THREE);
                break;
            }

            //Strafing.
            double speed = .2;
            leftRear.setPower(-speed);
            leftFront.setPower(speed);
            rightRear.setPower(speed);
            rightFront.setPower(-speed);
        }
        if(isStopRequested())
            return;
        _foundGold = true;
        if (System.currentTimeMillis() - 1500 < time && vuforia.seesGoldLeft(930,100)) {
            //telemetry.addData("Gold was at square", "1");
            //telemetry.update();
            override = true;
            smackAndRun(100, MineralSquare.ONE);
            return;
        }
        if (!override) {
            //telemetry.addData("Gold was at square", "2");
            //telemetry.update();
        }
    }

    public void strafeSquareUp() {

        if (_squaredOnGold || override) {
            return;
        }
        telemetry.addData("Strafe Square Up", "");
        telemetry.update();
        int pixels = 15;
        Recognition rec = vuforia.getGoldRecognition();
        if (rec != null) {
            int imgMid = rec.getImageWidth() / 2;
            int blockMid = (int) ((rec.getLeft() + rec.getRight()) / 2);
            //telemetry.addData("Image Middle: ", imgMid);
            //telemetry.addData("Block Middle: ", blockMid);
            //telemetry.update();
            int difff = blockMid - imgMid;
            if (Math.abs(difff) < pixels) {
                _squaredOnGold = true;
                //telemetry.addData("In", "range");
                //telemetry.update();
                leftRear.setPower(0);
                leftFront.setPower(0);
                rightRear.setPower(0);
                rightFront.setPower(0);
                sleep(100);
                smackAndRun(200, MineralSquare.TWO);
                return;
            }
            double speed = .15;
            if (difff > 0)
                speed = -speed;
            leftRear.setPower(-speed);
            leftFront.setPower(speed);
            rightRear.setPower(speed);
            rightFront.setPower(-speed);
        } else {
            telemetry.addData("Rec is", "null");
            telemetry.update();
        }
    }

    public void smackAndRun(int retreatDistance, MineralSquare mineralPos) {
        if(isStopRequested())
            return;
        telemetry.addData("Smack and Run", "");
        telemetry.update();
        int turn;
        rightRear.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        //Smacks the block and retreats.
        RunMotorsToPosition(500, .5);
        RunMotorsToPosition(retreatDistance, -1);
        //Determines how to turn.
        if (this.side == LanderSide.SILVER) {
            turn = 70;
            while (getImuAxis(Z) <= turn && !isStopRequested()) {
                turn(1);
            }
            if(isStopRequested())
                return;
            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
        } else {
            turn = 70;
            while (getImuAxis(Z) > -turn && !isStopRequested()) {
                turn(-1);
            }
            if(isStopRequested())
                return;
            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
        }
        if (this.side == LanderSide.GOLD) {
            switch (mineralPos) {
                case THREE:
                    RunMotorsToPosition(576, 1);//2 ROTATIONS
                    break;
                case ONE:
                    RunMotorsToPosition(288, 1); //1 ROTATION
                    break;
                case TWO:
                    RunMotorsToPosition(432, 1);
            }
        } else {
            switch (mineralPos) {
                case THREE:
                    RunMotorsToPosition(288, 1); //1 ROTATION
                    break;
                case ONE:
                    RunMotorsToPosition(576, 1);//2 ROTATIONS
                    break;
                case TWO:
                    RunMotorsToPosition(420, 1);
            }
        }
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        alignOnVumark();
    }

    public void alignOnVumark() {
        if(isStopRequested())
            return;
        telemetry.addData("Align on VuMark", "");
        telemetry.update();
        if (_alignedOnVumark) {
            return;
        }
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (side == LanderSide.GOLD) {
            while (vuforia.getPos()[0] == 120 && !isStopRequested()) {
                turn(.1);
            }
            if(isStopRequested())
                return;
            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
        } else {
            while (vuforia.getPos()[0] == 120 && !isStopRequested()) {
                turn(-.1);
            }
            if(isStopRequested())
                return;
            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
        }
        // we know where we are. orient
        double tAngle = 165;
        if (vuforia.getPos()[1] < 0) {
            tAngle = -15;
        }
        double dTheta = tAngle - vuforia.getRotation().thirdAngle;
        double imuTargetAngle = getImuAxis(Z) + dTheta;
        double[] pos = vuforia.getPos();
        double currX = pos[0];
        double currY = pos[1];
        // now turn left until our current angle is >= target angle
        while (getImuAxis((Z)) < imuTargetAngle && !isStopRequested()) {
            turn(0.3);
            //telemetry.addData("IMU Z", getImuAxis(Z));
            //telemetry.addData("Target", imuTargetAngle);
            //telemetry.update();
        }
        if(isStopRequested())
            return;
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        throwSirQuacksalot(currX, currY);
        _alignedOnVumark = true;
    }

    public void throwSirQuacksalot(double currX, double currY) {
        if(isStopRequested())
            return;
        telemetry.addData("Throw Sir Quacksalot", "");
        telemetry.update();
        double speed = -1.0;
        double yDist = 55 - currY;
        if (currY < 0) {
            yDist = Math.abs(-55 - currY);
        }
        double xDist = Math.abs(-40 - currX);
        if (currY < 0) {
            xDist = 40 - currX;
        }
        //For some reason, currY is adding another 12 inches.
        if (currX > 0 && currY < 0) {
            yDist += 12;
        } else if (currX < 0 && currY > 0) {
            yDist += 12;
        } else {
            yDist -= 3;
        }
        // assume we strafe at 12 inches per second
        double t = System.currentTimeMillis();
        int time = (int) ((yDist / 11) * 1000);
        while (t + time > System.currentTimeMillis() && !isStopRequested()) {
            //telemetry.addData("yDist", yDist);
            //telemetry.addData("xDist", xDist);
            //telemetry.addData("Time", time);
            //telemetry.update();
            leftRear.setPower(-speed);
            leftFront.setPower(speed);
            rightRear.setPower(speed);
            rightFront.setPower(-speed);
        }
        if(isStopRequested())
            return;
        if (side == LanderSide.GOLD) {
            while (getImuAxis(Z) < 45 && !isStopRequested()) {
                turn(0.3);
            }
            if(isStopRequested())
                return;
        } else {
            while (getImuAxis(Z) < 135 && !isStopRequested()) {
                turn(0.3);
            }
            if(isStopRequested())
                return;
        }
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        double rotationsNeeded = xDist / (4 * Math.PI);
        int ticks = (int) (rotationsNeeded * 288);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RunMotorsToPosition(ticks, 1);
        duckarm.setPosition(1);
        while (getImuAxis(Z) > -130 && !isStopRequested()) {

            turn(-0.5);
        }
        if(isStopRequested())
            return;
        int newpos = (int) (40 / (Math.PI * 4.0) * 288);
        RunMotorsToPosition(newpos, 1.0);
        arm.setPosition(0);

    }

    public double getImuAxis(Axis axis) {
        Orientation o = imu.getAngularOrientation();
        AxesOrder axesOrder = o.axesOrder;
        double x;
        double z;
        double y;

        switch (axesOrder) {
            // We're not going to be using the Z or Y axis. If the robot is rotating along the Z or Y axis, we have a problem....
            case XYZ:
                x = o.firstAngle;
                z = o.secondAngle;
                y = o.thirdAngle;
                break;

            case XZY:
                x = o.firstAngle;
                z = o.thirdAngle;
                y = o.secondAngle;
                break;

            case YXZ:
                x = o.secondAngle;
                z = o.firstAngle;
                y = o.thirdAngle;
                break;

            case ZXY:
                x = o.secondAngle;
                z = o.firstAngle;
                y = o.thirdAngle;
                break;

            case YZX:
                x = o.thirdAngle;
                z = o.secondAngle;
                y = o.firstAngle;
                break;

            case ZYX:
                x = o.thirdAngle;
                z = o.firstAngle;
                y = o.secondAngle;
                break;

            default:
                x = o.firstAngle;
                z = o.secondAngle;
                y = o.thirdAngle;
                break;
        }
        switch (axis) {
            case X:
                return x;
            case Y:
                return y;
            case Z:
                return z;
            default:
                return x;
        }
    }

    public double degreesToCounts(double degrees) {
        return (degrees * 288) / 360;
    }

    public enum Axis {
        X,
        Y,
        Z,
    }

    public enum MineralSquare {
        THREE,
        TWO,
        ONE
    }
}

