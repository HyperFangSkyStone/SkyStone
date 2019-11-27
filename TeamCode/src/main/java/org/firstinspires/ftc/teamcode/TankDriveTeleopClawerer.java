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
        /*tankDrive.LClaw.setPosition(.3);
        tankDrive.RClaw.setPosition(.5);
        tankDrive.PosClaw.setPosition(1);
        */
        telemetry.addData("R Claw :: ", tankDrive.RClaw.getPosition());
        telemetry.addData("L Claw :: ", tankDrive.LClaw.getPosition());
        telemetry.addData("Pos Claw :: ", tankDrive.PosClaw.getPosition());
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad2.dpad_right) {
                tankDrive.RClaw.setPosition(tankDrive.RClaw.getPosition() + .03);
                tankDrive.LClaw.setPosition(tankDrive.LClaw.getPosition() - .03);
                telemetry.addData("R Claw CLAMP :: ", tankDrive.RClaw.getPosition());
                telemetry.addData("L Claw CLAMP :: ", tankDrive.LClaw.getPosition());
                telemetry.update();
                sleep(100);
            }
            else if (gamepad2.dpad_left) {
                tankDrive.RClaw.setPosition(tankDrive.RClaw.getPosition() - .03);
                tankDrive.LClaw.setPosition(tankDrive.LClaw.getPosition() + .03);
                telemetry.addData("R Claw LETGO :: ", tankDrive.RClaw.getPosition());
                telemetry.addData("L Claw LETGO :: ", tankDrive.LClaw.getPosition());
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

            if (gamepad2.a) {
                tankDrive.RClaw.setPosition(0.4);
                tankDrive.LClaw.setPosition(0.4);
            } else if (gamepad2.b) {
                tankDrive.RClaw.setPosition(0.25);
                tankDrive.LClaw.setPosition(0.55);
            }


        }
        //freeze();

    }
}
