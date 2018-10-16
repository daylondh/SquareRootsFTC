package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.util.ArrayList;
import java.util.List;


@TeleOp
public class PixyExample extends LinearOpMode {
    I2cDeviceSynch pixy;

    @Override
    public void runOpMode() throws InterruptedException {
        //setting up Pixy to the hardware map
        pixy = hardwareMap.i2cDeviceSynch.get("pixy");
        // ALEX: I put this arrayList in here because it'll simplify things for later.
        // This way, if you want signature 2 but not 1, or l and 3 (or anything else), you can get
        // them this way.
        List<Byte> desiredSignatures = new ArrayList<Byte>();
        desiredSignatures.add((byte) 1); // we want at least the first signature...

        waitForStart();

        while(opModeIsActive()) {
            pixy.engage();
            CheckAllSigs(desiredSignatures);
            telemetry.update();
        }
    }

    private void CheckAllSigs(List<Byte> desiredSignatures) {
        for (Byte b : desiredSignatures) {
            byte[] signature = pixy.read(0x50 + b, 5);
            int x = 0xff & signature[1];
            int y = 0xff & signature[2];
            int width = 0xff & signature[3];
            int height = 0xff & signature[4];
            telemetry.addData("Signature ", b);
            telemetry.addData("X: ", x);
            telemetry.addData("Y: ", y);
            telemetry.addData("Width: ", width);
            telemetry.addData("Height: ", height);
        }
    }
}