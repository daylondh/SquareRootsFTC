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
    double displacement = 1.0;
    private Servo cameraServo;
    boolean isPressed;
    private Servo leftTooth;
    private Servo rightTooth;
    private Servo wrist;
    private DcMotor shoulder;
    private double servoPos = 0;
    static final double COUNTS_PER_MOTOR_REV = 288;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    boolean justMovedSlower = false;
    boolean justMovedFaster = false;

    @Override
    public void runOpMode() {
        //Setup driving stuffs.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        instantiateActuators();
        int startPos = lift.getCurrentPosition();


        waitForStart();
        runtime.reset();

        justMovedSlower = false;
        justMovedFaster = false;

        while (opModeIsActive()) {

            HandleGamepad1();

            HandleGamepad2();

            // Show the elapsed game time and wheel power.
            int rotationDist = lift.getCurrentPosition();
            telemetry.addData("Lift: ", rotationDist - startPos);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Displacement:", displacement);

            telemetry.update();
        }
    }

    private void HandleGamepad1() {
        boolean goUp = gamepad1.a;
        boolean goDown = gamepad1.b;
        double liftPower = 1;

        // x y = wrist. bumper and trigger have teeth. left stick is shoulder.
        if (gamepad1.right_bumper && !justMovedFaster) {
            displacement = displacement + .1;
            if (displacement > 1.0)
                displacement = 1.0;
            justMovedFaster = true;
        }


        if (gamepad1.left_bumper && !justMovedSlower) {
            displacement = displacement - .1;
            if (displacement < 0.5)
                displacement = 0.5;
            justMovedSlower = true;
        }
        if (!gamepad1.right_bumper) {
            justMovedFaster = false;
        }
        if (!gamepad1.left_bumper) {
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
        //determine if we should turn. Turn has priority over strafe.
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
        telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftRear (%.2f), rightRear (%.2f)", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
    }

    private void HandleGamepad2() {

        if (gamepad2.left_bumper) {
            leftTooth.setPosition(1); //Positive number makes left close.
        }
        if (gamepad2.left_trigger != 0) {
            leftTooth.setPosition(-1);
        }
        if (gamepad2.right_bumper) {
            rightTooth.setPosition(-1); //Negative number makes right close.
        }
        if (gamepad2.right_trigger != 0) {
            rightTooth.setPosition(1);
        }
        if (gamepad2.x) {//Run wrist negative to open.
            wrist.setPosition(-1);
        }
        if (gamepad2.y) { // X opens, Y retracts.
            wrist.setPosition(1);
        }
        if (gamepad2.left_stick_y != 0) {
            shoulder.setPower(Range.clip(gamepad2.left_stick_y, -1, 1));
            //Sets the power accordingly to the left stick.
        }
    }

    private void instantiateActuators() {
        //Maps everything to the corresponding hardwaremap name, and sets the directions and makes the motors run with encoders.
        leftTooth = hardwareMap.get(Servo.class, "leftTooth");
        rightTooth = hardwareMap.get(Servo.class, "rightTooth");
        wrist = hardwareMap.get(Servo.class, "wrist");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        lift = hardwareMap.get(DcMotor.class, "lift");
        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        leftTooth = hardwareMap.get(Servo.class, "leftTooth");
        rightTooth = hardwareMap.get(Servo.class, "rightTooth");
        wrist = hardwareMap.get(Servo.class, "wrist");

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
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    private void wristTest() {

        if (gamepad1.left_trigger > 0) {
            servoPos -= 20;
            cameraServo.setPosition(servoPos);
        }
        if (gamepad1.right_trigger > 0) {
            servoPos += 20;
            cameraServo.setPosition(servoPos);

        }

    }

}
