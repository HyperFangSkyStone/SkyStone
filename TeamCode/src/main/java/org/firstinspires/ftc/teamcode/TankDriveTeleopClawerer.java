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
        tankDrive.LClaw.setPosition(.5);
        tankDrive.RClaw.setPosition(.25);
        tankDrive.PosClaw.setPosition(.75);
        telemetry.addData("R Claw :: ", tankDrive.RClaw.getPosition());
        telemetry.addData("L Claw :: ", tankDrive.LClaw.getPosition());
        telemetry.addData("Pos Claw :: ", tankDrive.PosClaw.getPosition());
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad2.dpad_right) {
                tankDrive.RClaw.setPosition(tankDrive.RClaw.getPosition() + .1);
                tankDrive.LClaw.setPosition(tankDrive.LClaw.getPosition() - .1);
                telemetry.addData("R Claw CLAMP :: ", tankDrive.RClaw.getPosition());
                telemetry.addData("L Claw CLAMP :: ", tankDrive.LClaw.getPosition());
                telemetry.update();
                sleep(100);
            }
            else if (gamepad2.dpad_left) {
                tankDrive.RClaw.setPosition(tankDrive.RClaw.getPosition() - .1);
                tankDrive.LClaw.setPosition(tankDrive.LClaw.getPosition() + .1);
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


        }
        //freeze();

    }
}
