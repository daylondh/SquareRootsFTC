package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class GoStraight extends SquareRootsAutonomousBase {

    private boolean ranitonce = false;

    public GoStraight() {
        super(AllianceSide.GOLD);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        waitForStart();
        while (opModeIsActive()) {
            if (!ranitonce) {
                goStraight(300, 1, 0);
                ranitonce = true;
            }

        }
    }
}
