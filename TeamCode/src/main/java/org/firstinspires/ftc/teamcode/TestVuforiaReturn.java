package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class TestVuforiaReturn extends LinearOpMode {
    SquareRootsVuforia vuforia;


    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforia = new SquareRootsVuforia(cameraMonitorViewId);
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforia.InitTfod(tfodMonitorViewId);
        waitForStart();
        while (opModeIsActive()) {
            double[] pos = vuforia.getPos();
            double[] rotation = vuforia.getRotation();
            telemetry.addData("Position: " + pos[0] + ", " + pos[1] + ", " + pos[2], null);
            telemetry.addData("Rotation: " + rotation[0] + ", " + rotation[1] + ", " + rotation[2], null);
            telemetry.update();
        }
    }

}
