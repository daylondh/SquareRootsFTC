package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class TestOpMode extends SquareRootsAutonomousBase {
    public TestOpMode() {
        super(LanderSide.SILVER);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("TensorFlow Data", vuforia.getGoldRecognition().getLeft());
            telemetry.update();
        }
    }

}
