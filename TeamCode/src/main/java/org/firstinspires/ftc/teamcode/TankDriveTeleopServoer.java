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
                telemetry.addData("Pservo1 Position", tankDrive.PServo1.getPosition()); //To be replaced with the correct servo
                telemetry.addData("Pservo2 Position", tankDrive.PServo2.getPosition()); //To be replaced with the correct servo
                telemetry.update();
            }
        }

        //freeze();

    }
}
