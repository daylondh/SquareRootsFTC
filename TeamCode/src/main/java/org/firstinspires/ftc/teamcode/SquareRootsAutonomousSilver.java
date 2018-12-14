package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class SquareRootsAutonomousSilver extends SquareRootsAutonomousBase {
    //Tell superclass that we are gold.
    public SquareRootsAutonomousSilver() {
        super(LanderSide.SILVER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        waitForStart();
        while (opModeIsActive()) {
            DoLanding();
            ClearLander();
            strafeToGetGold();
            strafeSquareUp();
        }
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
    }
}
