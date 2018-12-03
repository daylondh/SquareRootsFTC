package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class TestOpMode extends LinearOpMode {
    SquareRootsVuforia vuforia;
    private boolean _armDeployed;
    protected DcMotor shoulder;
    private Servo wrist;


    @Override
    public void runOpMode() throws InterruptedException {
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist = hardwareMap.get(Servo.class, "wrist");
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.a) {
                DeployArm();
            }
            if (gamepad2.b)
            {
                HomeArm();
            }
            if(_armDeployed)
                break;

        }
    }

    private void HomeArm() {
        shoulder.setTargetPosition(10);
        shoulder.setPower(-1.0);
        //wrist.setPosition(10);
    }

    private void DeployArm() {
        if(_armDeployed)
            return;
        wrist.setPosition(-1);
        sleep(1500);
        shoulder.setTargetPosition(130);
        shoulder.setPower(1.0);
        //_armDeployed = true;
    }

}
