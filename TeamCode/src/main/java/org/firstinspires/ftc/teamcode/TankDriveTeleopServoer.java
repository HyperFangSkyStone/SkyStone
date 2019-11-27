package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="Tank Servoer", group="1")
//@Disabled
public class TankDriveTeleopServoer extends LinearOpMode {

    TankDriveALPHA tankDrive = new TankDriveALPHA();
    @Override
    public void runOpMode() throws InterruptedException
    {
        tankDrive.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad2.a) {
                tankDrive.PServo1.setPower(.5);
                tankDrive.PServo2.setPower(-.5);
            } else if (gamepad2.b) {
                tankDrive.PServo1.setPower(-.5);
                tankDrive.PServo2.setPower(.5);
            } else {
                tankDrive.PServo1.setPower(0);
                tankDrive.PServo1.setPower(0);
            }
        }

        //freeze();

    }
}
