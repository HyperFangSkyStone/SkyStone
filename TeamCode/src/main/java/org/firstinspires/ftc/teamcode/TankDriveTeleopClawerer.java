package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Tank Clawerer", group="1")
//@Disabled
public class TankDriveTeleopClawerer extends LinearOpMode {

    TankDriveALPHA tankDrive = new TankDriveALPHA();
    @Override
    public void runOpMode() throws InterruptedException
    {
        tankDrive.init(hardwareMap);
        tankDrive.LClaw.setPosition(0);
        tankDrive.RClaw.setPosition(0);
        tankDrive.PosClaw.setPosition(0);
        telemetry.addData("All", "Done");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad2.x)
            {
                tankDrive.LClaw.setPosition(.5);
                tankDrive.RClaw.setPosition(.5);
            }
            if (gamepad2.y)
            {
                tankDrive.PosClaw.setPosition(.5);
            }


            telemetry.addData("Left Claw:", tankDrive.LClaw.getPosition());
            telemetry.addData("Right Claw:", tankDrive.RClaw.getPosition());
            telemetry.addData("Pos Claw:", tankDrive.PosClaw.getPosition());
            telemetry.update();
        }

        //freeze();

    }
}
