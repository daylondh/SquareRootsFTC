package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class SquareRootsAutonomousGold extends SquareRootsAutonomousBase {
    //Tell superclass that we are gold.
    public SquareRootsAutonomousGold() {
        super(AllianceSide.GOLD);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        waitForStart();
        while (opModeIsActive()) {
            //DoLanding();
            //ClearLander();
            strafeToGetGold();
            strafeSquareUp();
        }
    }
}
