package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GoToPosition {

    private final SquareRootsVuforia vuforia;
    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final DcMotor leftRear;
    private final DcMotor rightRear;
    private final BNO055IMU imu;
    private Telemetry telemetry;


    public GoToPosition(SquareRootsVuforia squareRootsVuforia, DcMotor leftFront, DcMotor rightFront, DcMotor leftRear, DcMotor rightRear, BNO055IMU imu, Telemetry telemetry)
    {
        vuforia = squareRootsVuforia;
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;
        this.imu = imu;
        this.telemetry = telemetry;
    }

    private void Localize(TurnDirection dir) {
        while(vuforia.getPos()[0] == 120.0)
        {
            // actually localize
            double turnspeed = 0.1;
            turnspeed *= dir == TurnDirection.RIGHT? -1:1;
            leftFront.setPower(turnspeed);
            rightFront.setPower(-turnspeed);
            leftRear.setPower(turnspeed);
            rightRear.setPower(-turnspeed);
        }
        double turnspeed = 0;
        leftFront.setPower(turnspeed);
        rightFront.setPower(-turnspeed);
        leftRear.setPower(turnspeed);
        rightRear.setPower(turnspeed);
    }

    private void Reorient(double x, double y) {

        double[] currPos = vuforia.getPos();
        Orientation vuforiaOrientation = vuforia.getRotation().toAngleUnit(AngleUnit.DEGREES);
        double zVuforia = vuforiaOrientation.thirdAngle; // it goes XYZ
        double dx = x - currPos[0];
        double dy = y - currPos[1];
        double tc = Math.toDegrees(Math.atan2(dy, dx));
        double dt = zVuforia-tc;

        Orientation imuOrientation = imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES);
        double zImu = imuOrientation.firstAngle; // it goes ZXY
        double targetAngle = zImu + dt;
        while(targetAngle > 180)
            targetAngle -= 180;
        while(targetAngle < -180)
            targetAngle += 180;
        while(Math.abs(zImu - targetAngle) > 5)
        {
            zImu = imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle;
            double turnspeed = (targetAngle - zImu)/90;
            if(turnspeed > 1)
                turnspeed = 1;
            if(turnspeed < -1)
                turnspeed = -1;
            leftFront.setPower(-turnspeed);
            rightFront.setPower(turnspeed);
            leftRear.setPower(-turnspeed);
            rightRear.setPower(turnspeed);
            telemetry.addData("IMU: " + zImu + ", Target", targetAngle);
            telemetry.update();

        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

    }

    private double CalculateDistance(double x, double y) {
        return 0;
    }

    public void Go(double x, double y, TurnDirection dir) {
        Localize(dir);
        Reorient(x, y);
        double dist = CalculateDistance(x, y);
        // now go
    }
}
