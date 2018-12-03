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

import java.util.List;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.SquareRootsAutonomousBase.Axis.Y;
import static org.firstinspires.ftc.teamcode.SquareRootsAutonomousBase.Axis.Z;

public abstract class SquareRootsAutonomousBase extends LinearOpMode {

    public List<PixySignature> sigs;
    protected boolean _landingDone;
    protected DcMotor leftFront;
    protected DcMotor rightFront;
    protected DcMotor rightRear;
    protected DcMotor leftRear;
    protected DcMotor lift;
    protected DcMotor arm;
    protected BNO055IMU imu;
    protected SquareRootsVuforia vuforia;
    protected PixyManager pixy;
    protected boolean _clearedLander;
    protected Servo duckarm;
    protected Servo leftTooth;
    protected Servo rightTooth;
    protected Servo wrist;
    protected DcMotor shoulder;
    protected GoToPosition pos;
    AllianceSide side;
    private boolean _foundGold;
    private boolean _squaredOnGold = false;
    private boolean override;
    private boolean _alignedOnVumark;

    public SquareRootsAutonomousBase(AllianceSide side) {
        this.side = side;
    }

    protected void Init() {
        //Maps motors and servos to their corresponding position on the hardwaremap,
        // updates the telemetry, sets up vuforia, sets the motors to run with encoder.

        telemetry.addData("Initializing", "true");
        telemetry.update();
        _landingDone = false;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        lift = hardwareMap.get(DcMotor.class, "lift");
        pixy = new PixyManager(hardwareMap.i2cDeviceSynch.get("pixy"));
        leftTooth = hardwareMap.get(Servo.class, "leftTooth");
        rightTooth = hardwareMap.get(Servo.class, "rightTooth");
        wrist = hardwareMap.get(Servo.class, "wrist");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        duckarm = hardwareMap.get(Servo.class, "duckArm");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforia = new SquareRootsVuforia(cameraMonitorViewId);
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforia.InitTfod(tfodMonitorViewId);


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

        pos = new GoToPosition(vuforia, leftFront, rightFront, leftRear, rightRear, imu, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    protected void DoLanding() {
        //Does the landing.
        // TODO: 11/13/2018 Add a failsafe. If the drive team used b to go up, this will NOT WORK!!!
        if (_landingDone)
            return;
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(7400);
        lift.setPower(1);
        while (lift.isBusy()) {
            sleep(10);
        }
        lift.setPower(0);

        double strafe = -1.0;
        double leftFrontPower = -strafe;
        double rightFrontPower = strafe;
        double leftRearPower = strafe;
        double rightRearPower = -strafe;


        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
        sleep(900);
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
        duckarm.setPosition(2.0);
        int targetPos = 70;
        double targetSpeed = 0.8;
        turn(-1.0);
        sleep(500);
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

    public void strafe(int distance, double speed) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Sets all the motors to run with encoders, so that we can measure the distance the motor has gone.
        leftFront.setPower(-speed);
        leftRear.setPower(speed);
        rightFront.setPower(speed);
        rightRear.setPower(-speed);
        // Run motors at target speed.
        while (leftFront.getCurrentPosition() < distance || rightFront.getCurrentPosition() < distance
                || leftRear.getCurrentPosition() < distance || rightRear.getCurrentPosition() < distance) {
            //This loop continuously tests to see if the individual motors have reached the target position.
            sleep(10);
            if (leftFront.getTargetPosition() >= distance)
                leftFront.setPower(0);
            if (rightFront.getTargetPosition() >= distance)
                rightFront.setPower(0);
            if (leftRear.getTargetPosition() >= distance)
                leftRear.setPower(0);
            if (rightRear.getTargetPosition() >= distance)
                rightRear.setPower(0);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    protected void turn(double turnScale) {
        // Assuming that turnScale isn't 0, it doesn't matter what you put in, since  --x = x.
        double clippedTurnScale = Range.clip(turnScale, -1.0, 1.0);
        rightRear.setPower(-clippedTurnScale);
        rightFront.setPower(-clippedTurnScale);
        leftFront.setPower(clippedTurnScale);
        leftRear.setPower(clippedTurnScale);
    }

    protected void strafeToGetGold() {
        if (_foundGold) {
            return;
        }
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        long time = System.currentTimeMillis();
        while (!vuforia.seesGold(930) && opModeIsActive()) {
            telemetry.addData("TensorFlow Data: ", vuforia.getGoldRecognition());
            telemetry.update();
            // Wait 8 seconds, if it hasn't seen the gold, it gives up.
            if (System.currentTimeMillis() - 7000 > time) {
                telemetry.addData("Giving up", null);
                telemetry.update();
                override = true;
                smackAndRun(100, MineralSquare.THREE);
                break;
            }

            //Strafing.
            double speed = .15;
            leftFront.setPower(-speed);
            leftRear.setPower(speed);
            rightFront.setPower(speed);
            rightRear.setPower(-speed);
        }
        _foundGold = true;
        if (System.currentTimeMillis() - 1500 < time) {
            telemetry.addData("Gold was at square", "1");
            telemetry.update();
            override = true;
            smackAndRun(100, MineralSquare.ONE);
            return;
        }
        if (!override) {
            telemetry.addData("Gold was at square", "2");
            telemetry.update();
        }
    }

    public void strafeSquareUp() {

        if (_squaredOnGold || override) {
            return;
        }
        //    if (getGoldNumber() == MineralNumber.GOLD_ONE) {
        //
        //            telemetry.addData("GOLD ONE", null);
        //            telemetry.update();
        //            double speed = .15;
        //            leftFront.setPower(-speed);
        //            leftRear.setPower(speed);
        //            rightFront.setPower(speed);
        //            rightRear.setPower(-speed);
        //            sleep(600);
        //        }
        //        if (getGoldNumber() == MineralNumber.GOLD_THREE) {
        //
        //            telemetry.addData("GOLD THREE", null);
        //            telemetry.update();
        //            double speed = -.15;
        //            leftFront.setPower(-speed);
        //            leftRear.setPower(speed);
        //            rightFront.setPower(speed);
        //            rightRear.setPower(-speed);
        //            sleep(600);
        //        }


        int pixels = 15;
        Recognition rec = vuforia.getGoldRecognition();
        if (rec != null) {
            int imgMid = rec.getImageWidth() / 2;
            int blockMid = (int) ((rec.getLeft() + rec.getRight()) / 2);
            telemetry.addData("IMage Middle: ", imgMid);
            telemetry.addData("BLock Middle: ", blockMid);
            telemetry.update();
            int difff = blockMid - imgMid;
            if (Math.abs(difff) < pixels) {
                _squaredOnGold = true;
                telemetry.addData("In", "range");
                telemetry.update();
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(100);
                smackAndRun(200, MineralSquare.TWO);
                return;
            }
            double speed = .15;
            if (difff > 0)
                speed = -speed;
            leftFront.setPower(-speed);
            leftRear.setPower(speed);
            rightFront.setPower(speed);
            rightRear.setPower(-speed);
        } else {
            telemetry.addData("Rec is", "null");
            telemetry.update();
        }
    }

    public void smackAndRun(int retreatDistance, MineralSquare d) { // TODO: 11/28/2018 MAKE TURN HAVE GREATER ANGLE FOR LINING UP.
        int turn;
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
        //Smacks the block and retreats.
        RunMotorsToPosition(500, .5);
        RunMotorsToPosition(retreatDistance, -1);
        //Determines how to turn.
        if (this.side == AllianceSide.SILVER) {
            switch (d) {
                case THREE:
                    turn = 45;
                    break;
                case TWO:
                    turn = 55;
                    break;
                case ONE:
                    turn = 65;
                    break;
                default:
                    turn = 45;

            }
            while (getImuAxis(Y) <= turn) {
                turn(1);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        } else {
            switch (d) {
                case THREE:
                    turn = 65;
                    break;
                case TWO:
                    turn = 55;
                    break;
                case ONE:
                    turn = 45;
                    break;
                default:
                    turn = 65;

            }
            while (getImuAxis(Y) > -turn) {
                turn(-1);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        }
        if (this.side == AllianceSide.GOLD) {
            switch (d) {
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
            switch (d) {
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
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        alignOnVumark();
    }

    public void alignOnVumark() { // TODO: 11/30/2018 Make it turn according to the side.

        if (_alignedOnVumark) {
            return;
        }
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (side == AllianceSide.GOLD) {
            while (vuforia.getPos()[0] == 120) {
                turn(.1);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        } else {
            while (vuforia.getPos()[0] == 120) {
                turn(-.1);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
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
        while (getImuAxis((Z)) < imuTargetAngle) {
            turn(0.3);
            telemetry.addData("IMU Z", getImuAxis(Z));
            telemetry.addData("Target", imuTargetAngle);
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        throwSirQuacksalot(currX, currY);
        _alignedOnVumark = true;
    }

    public void throwSirQuacksalot(double currX, double currY) {
        double speed = -1.0;
        double yDist = 55 - currY;
        if (currY < 0) {
            yDist = Math.abs(-55 - currY);
        }
        double xDist = Math.abs(-40 - currX);
        if (currY < 0) {
            xDist = 40 - currX;
        }
        //For some reason, currY was adding another 12 inches.
        if (currX > 0 && currY < 0) {
            yDist += 12;
        } else if (currX < 0 && currY > 0) {
            yDist += 12;
        } else {
            yDist -= 3;
        }
        // assume we strafe at 12 inches per second
        double t = System.currentTimeMillis();
        int time = (int) ((yDist / 12) * 1000);
    /*    while(true)
        {
            telemetry.addData("xVal", currX);
            telemetry.addData("yVal", currY);
            telemetry.addData("xDist", xDist);
            telemetry.addData("yDist", yDist);
            telemetry.addData("Time", time);
            telemetry.update();
            }
         */
        while (t + time > System.currentTimeMillis()) {
            telemetry.addData("yDist", yDist);
            telemetry.addData("xDist", xDist);
            telemetry.addData("Time", time);
            telemetry.update();
            leftFront.setPower(-speed);
            leftRear.setPower(speed);
            rightFront.setPower(speed);
            rightRear.setPower(-speed);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        double rotationsNeeded = xDist / (4 * Math.PI);
        int ticks = (int) (rotationsNeeded * 288);
        telemetry.addData("yDist", yDist);
        telemetry.addData("Rotations", rotationsNeeded);
        telemetry.addData("Ticks", ticks);
        telemetry.update();
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RunMotorsToPosition(ticks, 1);
        duckarm.setPosition(2);
    }

    public void goStraight(double distance, double targetSpeed, int line) {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double lfpeed = targetSpeed;
        double rfspeed = targetSpeed;
        double lrspeed = targetSpeed;
        double rrspeed = targetSpeed;
        boolean isTurning = false;

        while (leftFront.getCurrentPosition() < distance || rightFront.getCurrentPosition() < distance
                || leftRear.getCurrentPosition() < distance || rightRear.getCurrentPosition() < distance) {
            //This loop continuously tests to see if the individual motors have reached the target position.
            sleep(10);

            double z = getImuAxis(Z);


            if (z > line + 1 && !isTurning) {
                // Turn right.
                lrspeed -= .5;
                lfpeed -= .5;
                telemetry.addData("Turing", "Right");
                telemetry.update();
                isTurning = true;
            }
            if (z < line - 1 && !isTurning) {

                rrspeed -= .5;
                rfspeed -= .5;
                telemetry.addData("Turing", "Left");
                telemetry.update();
                isTurning = true;
            }
            if (z > line - 1 && z < line + 1) {
                lfpeed = targetSpeed;
                rfspeed = targetSpeed;
                lrspeed = targetSpeed;
                rrspeed = targetSpeed;
                isTurning = false;
                telemetry.addData("Just", "Truckin' along");
                telemetry.update();
            }

            leftFront.setPower(lfpeed);
            rightFront.setPower(rfspeed);
            leftRear.setPower(lrspeed);
            rightRear.setPower(rrspeed);

            if (leftFront.getTargetPosition() >= distance)
                leftFront.setPower(0);
            if (rightFront.getTargetPosition() >= distance)
                rightFront.setPower(0);
            if (leftRear.getTargetPosition() >= distance)
                leftRear.setPower(0);
            if (rightRear.getTargetPosition() >= distance)
                rightRear.setPower(0);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        telemetry.update();
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

    public enum Axis {
        X,
        Y,
        Z,
    }

    public enum AllianceSide {
        GOLD,
        SILVER
    }

    public enum direction {
        STRAFE, DRIVE
    }

    public enum MineralSquare {
        THREE,
        TWO,
        ONE
    }
}

