package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TestOpMode extends LinearOpMode{
    private Gyroscope imu;
    private DcMotor motorTest;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo servoTest;

    @Override
    public void runOpMode() {
       imu = hardwareMap.get(Gyroscope.class, "imu");
       motorTest = hardwareMap.get(DcMotor.class, "motorTest");
       digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
       sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
       servoTest = hardwareMap.get(Servo.class, "servoTest");


       digitalTouch.setMode(DigitalChannel.Mode.INPUT);
       telemetry.addData("Status", "Initialized");
       telemetry.update();
       waitForStart();
        double targetPower = 0;
       while (opModeIsActive()) {
          targetPower = -this.gamepad1.left_stick_y;
          motorTest.setPower(targetPower);
          if (gamepad2.y) {
              servoTest.setPosition(0);
          }
          else if (gamepad2.x || gamepad2.b) {
              servoTest.setPosition(0.5);
          }
          else if (gamepad2.a) {
              servoTest.setPosition(1);
          }
          if (digitalTouch.getState()) {
              telemetry.addData("Button", "not pressed");
          }
          else {
              telemetry.addData("Button", "pressed");
          }
          telemetry.addData("Servo Position", servoTest.getPosition());
          telemetry.addData("Target Power", targetPower);
          telemetry.addData("Motor Power", motorTest.getPower());
          telemetry.addData("Distance", sensorColorRange.getDistance(DistanceUnit.CM));
          telemetry.addData("Status", "Running");
          telemetry.update();





       }
    }
}
