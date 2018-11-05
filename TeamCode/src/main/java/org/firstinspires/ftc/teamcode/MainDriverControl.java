package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class MainDriverControl extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor leftRear;
    private DcMotor lift;
    private DistanceSensor rangeSensor;
    double displacement = 1.0;
    private Servo cameraServo;
    boolean isPressed ;
    private double servoPos = 0;
    static final double COUNTS_PER_MOTOR_REV = 288;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        //Setup driving stuffs.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        lift = hardwareMap.get(DcMotor.class, "lift");
        rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeSensor");
        cameraServo = hardwareMap.get(Servo.class, "cameraServo");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        int startPos = lift.getCurrentPosition();

        waitForStart();
        runtime.reset();
        boolean justMovedSlower = false;
        boolean justMovedFaster = false;


        while (opModeIsActive()) {
            boolean goUp = gamepad1.a;
            boolean goDown = gamepad1.b;
            double liftPower = 1;

            if ( gamepad1.right_bumper && !justMovedFaster) {
                displacement = displacement + .1;
                if (displacement > 1.0)
                    displacement = 1.0;
                justMovedFaster = true;
            }


            if ( gamepad1.left_bumper && !justMovedSlower){
                displacement = displacement - .1;
                if(displacement < 0.5)
                    displacement = 0.5;
                justMovedSlower = true;
            }
            if(!gamepad1.right_bumper)
            {
                justMovedFaster = false;
            }
            if(!gamepad1.left_bumper)
            {
                justMovedSlower = false;
            }


            // Setup a variable for each drive wheel to save power level for telemetry
            double leftFrontPower = 0;
            double rightFrontPower = 0;
            double rightRearPower = 0;
            double leftRearPower = 0;


            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            //determine if we should turn.
            if (turn != 0) {
                leftFrontPower = Range.clip(drive - turn, -1.0, 1.0);
                leftRearPower = Range.clip(drive - turn, -1.0, 1.0);
                rightFrontPower = Range.clip(drive + turn, -1.0, 1.0);
                rightRearPower = Range.clip(drive + turn, -1.0, 1.0);

                rightFrontPower = rightFrontPower * displacement;
                leftRearPower = leftRearPower * displacement;
                rightFrontPower = rightFrontPower * displacement;
                rightRearPower = rightRearPower * displacement;

                leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0);
                leftRearPower = Range.clip(leftRearPower, -1.0, 1.0);
                rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0);
                rightRearPower = Range.clip(rightRearPower, -1.0, 1.0);

                leftFront.setPower(leftFrontPower);
                rightFront.setPower(rightFrontPower);
                leftRear.setPower(leftRearPower);
                rightRear.setPower(rightRearPower);
            } else {

                leftFrontPower = Range.clip(drive - strafe, -1.0, 1.0);
                rightFrontPower = Range.clip(drive + strafe, -1.0, 1.0);
                leftRearPower = Range.clip(drive + strafe, -1.0, 1.0);
                rightRearPower = Range.clip(drive - strafe, -1.0, 1.0);

                // Adds bumper trimming

                rightFrontPower = rightFrontPower * displacement;
                leftRearPower = leftRearPower * displacement;
                rightFrontPower = rightFrontPower * displacement;
                rightRearPower = rightRearPower * displacement;

                //Makes sure values don't go out of bounds
                leftFrontPower = Range.clip(leftFrontPower, -1.0, 1.0);
                leftRearPower = Range.clip(leftRearPower, -1.0, 1.0);
                rightFrontPower = Range.clip(rightFrontPower, -1.0, 1.0);
                rightRearPower = Range.clip(rightRearPower, -1.0, 1.0);




                leftFront.setPower(leftFrontPower);
                rightFront.setPower(rightFrontPower);
                leftRear.setPower(leftRearPower);
                rightRear.setPower(rightRearPower);
            }
            if (goDown) {
                lift.setPower(liftPower);

            } else if (goUp) {
                lift.setPower(-liftPower);
            } else {
                lift.setPower(0);
            }
            wristTest();


            // Show the elapsed game time and wheel power.
            int rotationDist = lift.getCurrentPosition();
            telemetry.addData("Lift: ", rotationDist - startPos);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftRear (%.2f), rightRear (%.2f)", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
            telemetry.addData("Displacement:", displacement);
            double dist = rangeSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance (cm)", dist);

        telemetry.update();
    }
    }
    private void wristTest(){

        if(gamepad1.left_trigger > 0) {
            servoPos -=20;
            cameraServo.setPosition(servoPos);
        }
        if (gamepad1.right_trigger > 0) {
            servoPos += 20;
            cameraServo.setPosition(servoPos);

        }

    }

}
