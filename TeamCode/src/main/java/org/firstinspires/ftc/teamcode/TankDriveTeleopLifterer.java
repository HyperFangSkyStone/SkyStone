package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Tank Lifterer", group="1")
//@Disabled
public class TankDriveTeleopLifterer extends LinearOpMode {

    TankDriveALPHA tankDrive = new TankDriveALPHA();
    @Override
    public void runOpMode() throws InterruptedException
    {
        tankDrive.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                tankDrive.Lift1.setPower(gamepad2.left_stick_y);
                tankDrive.Lift2.setPower(gamepad2.left_stick_y);
            }
            telemetry.addData("Lift 1:", tankDrive.Lift1.getCurrentPosition());
            telemetry.addData("Lift 2:", tankDrive.Lift2.getCurrentPosition());
            telemetry.update();
        }

        //freeze();

    }
}
