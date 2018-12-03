package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class SquareRootsAutonomousSilver extends SquareRootsAutonomousBase {
    //Tell superclass that we are gold.
    public SquareRootsAutonomousSilver() {
        super(SquareRootsAutonomousBase.AllianceSide.SILVER);
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
