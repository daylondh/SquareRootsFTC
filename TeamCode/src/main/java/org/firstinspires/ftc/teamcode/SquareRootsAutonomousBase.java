package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import static java.lang.Math.abs;

public abstract class SquareRootsAutonomousBase extends LinearOpMode {

    protected boolean _landingDone;
    protected DcMotor leftFront;
    protected DcMotor rightFront;
    protected DcMotor rightRear;
    protected DcMotor leftRear;
    protected DcMotor lift;
    protected DistanceSensor rangeSensor;
    protected Gyroscope imu;
    protected SquareRootsVuforia vuforia;
    protected PixyManager pixy;
    protected boolean _clearedLander;
    private boolean _hitGoldBlock;
    protected Servo cameraServo;
    private final int TARGET_WIDTH = 90;

    protected void Init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        _landingDone = false;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        lift = hardwareMap.get(DcMotor.class, "lift");
        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        pixy = new PixyManager(hardwareMap.i2cDeviceSynch.get("pixy"));

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
        if (_landingDone)
            return;
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(7000);
        lift.setPower(1); // positive power means go
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
        sleep(1000);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        sleep(10);

        _landingDone = true;
    }

    protected void ClearLander()
    {
        if(_clearedLander)
            return;
        int targetPos = 288;
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
        leftFront.setPower(targetSpeed);
        rightFront.setPower(targetSpeed);
        leftRear.setPower(targetSpeed);
        rightRear.setPower(targetSpeed);
        while (leftFront.getCurrentPosition() < targetPos || rightFront.getCurrentPosition() < targetPos || leftRear.getCurrentPosition() < targetPos || rightRear.getCurrentPosition() < targetPos) {
            sleep(10);
            if(leftFront.getTargetPosition() >= targetPos)
                leftFront.setPower(0);
            if(rightFront.getTargetPosition() >= targetPos)
                rightFront.setPower(0);
            if(leftRear.getTargetPosition() >= targetPos)
                leftRear.setPower(0);
            if(rightRear.getTargetPosition() >= targetPos)
                rightRear.setPower(0);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    protected void HitGoldBlock()
    {
        if(_hitGoldBlock)
            return;
        cameraServo.setPosition(90);
        sleep(1000);
        List<PixySignature> sigs = pixy.Run();
        int viewCenter = 160;
        int viewHeight = 200;
        PixySignature yellowSig = sigs.get(0);
        int diffY = yellowSig.y - viewHeight/2;
        while(yellowSig.width  < TARGET_WIDTH){

            sigs = pixy.Run();
            yellowSig = sigs.get(0);
            diffY = yellowSig.y - viewHeight/2;
            while (abs(diffY) > 1) {
                sigs = pixy.Run();
                yellowSig = sigs.get(0);
                diffY = yellowSig.y - viewHeight / 2;
                double turnScale = 1.0 * diffY / (viewHeight / 2);
                turn(turnScale);
                telemetry.addData("Size: " + sigs.get(0).width + ", " + sigs.get(0).height, null);
                telemetry.update();
            }
            RunMotorsToPosition(50, 0.5);
        }

        cameraServo.setPosition(-90);
        _hitGoldBlock = true;
    }

    protected void turn(double turnScale)
    {
        double clippedTurnScale = Range.clip(turnScale, -1.0, 1.0);
        rightRear.setPower(-clippedTurnScale);
        rightFront.setPower(-clippedTurnScale);
        leftFront.setPower(clippedTurnScale);
        leftRear.setPower(clippedTurnScale);
    }
}
