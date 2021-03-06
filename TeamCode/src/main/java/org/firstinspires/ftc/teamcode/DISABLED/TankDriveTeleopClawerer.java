package org.firstinspires.ftc.teamcode.DISABLED;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TankDriveALPHA;

@Disabled
@TeleOp(name="Tank Clawerer", group="1")
public class TankDriveTeleopClawerer extends LinearOpMode {

    TankDriveALPHA tankDrive = new TankDriveALPHA();
    @Override
    public void runOpMode() throws InterruptedException
    {/*
        tankDrive.init(hardwareMap);
        /*tankDrive.LeftClaw.setPosition(.3);
        tankDrive.RightClaw.setPosition(.5);
        tankDrive.PosClaw.setPosition(1);

        telemetry.addData("R Claw :: ", tankDrive.RightClaw.getPosition());
        telemetry.addData("L Claw :: ", tankDrive.LeftClaw.getPosition());
        //telemetry.addData("Pos Claw :: ", tankDrive.PosClaw.getPosition());
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad2.dpad_right) {
                tankDrive.RightClaw.setPosition(tankDrive.RightClaw.getPosition() + .03);
                tankDrive.LeftClaw.setPosition(tankDrive.LeftClaw.getPosition() - .03);
                telemetry.addData("R Claw CLAMP :: ", tankDrive.RightClaw.getPosition());
                telemetry.addData("L Claw CLAMP :: ", tankDrive.LeftClaw.getPosition());
                telemetry.update();
                sleep(100);
            }
            else if (gamepad2.dpad_left) {
                tankDrive.RightClaw.setPosition(tankDrive.RightClaw.getPosition() - .03);
                tankDrive.LeftClaw.setPosition(tankDrive.LeftClaw.getPosition() + .03);
                telemetry.addData("R Claw LETGO :: ", tankDrive.RightClaw.getPosition());
                telemetry.addData("L Claw LETGO :: ", tankDrive.LeftClaw.getPosition());
                telemetry.update();
                sleep(100);

            }
            else if (gamepad2.dpad_up) {
                tankDrive.PosClaw.setPosition(tankDrive.PosClaw.getPosition() + .1);
                telemetry.addData("Pos Claw TURN R :: ", tankDrive.PosClaw.getPosition());
                telemetry.update();
                sleep(100);

            }
            else if (gamepad2.dpad_down) {
                tankDrive.PosClaw.setPosition(tankDrive.PosClaw.getPosition() - .1);
                telemetry.addData("Pos Claw TURN L :: ", tankDrive.PosClaw.getPosition());
                telemetry.update();
                sleep(100);

            }

            telemetry.addData("PosClaw Position", tankDrive.PosClaw.getPosition());
            telemetry.update();

            if (gamepad2.b) {
                tankDrive.RightClaw.setPosition(TankDriveALPHA.RCLAW_RELEASE_POS);
                tankDrive.LeftClaw.setPosition(TankDriveALPHA.LCLAW_RELEASE_POS);
            } else if (gamepad2.a) {
                tankDrive.RightClaw.setPosition(TankDriveALPHA.RCLAW_GRIP_POS);
                tankDrive.LeftClaw.setPosition(TankDriveALPHA.LCLAW_GRIP_POS);
            }


        }
        //freeze();*/

    }
}
