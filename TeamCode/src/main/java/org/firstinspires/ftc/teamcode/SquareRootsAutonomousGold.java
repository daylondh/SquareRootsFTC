package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class SquareRootsAutonomousGold extends SquareRootsAutonomousBase {
    //Tell superclass that we are gold.
    public SquareRootsAutonomousGold() {
        super(LanderSide.GOLD);
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
