package org.firstinspires.ftc.teamcode.DISABLED;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TankDriveALPHA;

import java.util.ArrayList;

@TeleOp(name="Tank Servoer", group="1")
@Disabled
public class TankDriveTeleopServoer extends LinearOpMode {

    TankDriveALPHA tankDrive = new TankDriveALPHA();
    @Override
    public void runOpMode() throws InterruptedException
    {
        tankDrive.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {/*
            if (gamepad2.a) {
                tankDrive.LeftBall.setPower(.5);
                tankDrive.RightBall.setPower(-.5);
            } else if (gamepad2.b) {
                tankDrive.LeftBall.setPower(-.5);
                tankDrive.RightBall.setPower(.5);
            } else {
                tankDrive.LeftBall.setPower(0);
                tankDrive.LeftBall.setPower(0);
            }*/
        }

        //freeze();

    }
}
