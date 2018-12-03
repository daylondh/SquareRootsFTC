package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
            double pos1 = pos[1];
            Orientation or = vuforia.getRotation().toAngleUnit(AngleUnit.DEGREES);
            telemetry.addData("Orientation1", or.firstAngle); // 89
            telemetry.addData("Orientation2", or.secondAngle); // 0.3
            telemetry.addData("Orientation3", or.thirdAngle); // -145
            telemetry.addData("Pos 1", pos1);
            double sign = Math.signum(pos[1]);
            telemetry.addData("iosdgh", sign);
            telemetry.update();
        }
        vuforia.Shutdown();
    }

}
