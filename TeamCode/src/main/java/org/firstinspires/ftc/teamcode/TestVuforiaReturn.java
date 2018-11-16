package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


@TeleOp
public class TestVuforiaReturn extends LinearOpMode {
    SquareRootsVuforia vuforia;


    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforia = new SquareRootsVuforia(cameraMonitorViewId);
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforia.InitTfod(tfodMonitorViewId);
        waitForStart();
        while (opModeIsActive()) {
            double[] pos = vuforia.getPos();
            telemetry.addData("Position: " + pos[0] + ", " + pos[1] + ", " + pos[2], null);
            Recognition rec = vuforia.getGoldRecognition();
            if(rec != null) {
                if(rec.getWidth() < 200)
                    telemetry.addData("Width: " + rec.getWidth() + ", Height: "+ rec.getHeight() + ", Angle", vuforia.getGold());
                if(rec.getWidth() > 200 && rec.getHeight() < rec.getWidth())
                {
                    telemetry.addData("Angle", vuforia.getGold());
                }
            }
            telemetry.update();
        }
        vuforia.Shutdown();
    }

}
