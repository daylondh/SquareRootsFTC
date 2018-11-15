package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class SquareRootsAutonomousBlueGold extends SquareRootsAutonomousBase {
    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        waitForStart();
        while(opModeIsActive())
        {
            DoLanding();
            ClearLander();
            findTheGold();
            HitGoldBlock();
        }
    }
}
