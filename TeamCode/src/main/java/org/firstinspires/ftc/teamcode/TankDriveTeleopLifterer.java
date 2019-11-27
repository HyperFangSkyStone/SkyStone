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
            if (gamepad2.y) {
                int targ = 175; //Target
                double kp = 0.003;
                double powerFloor = 0.3;
                int errMarg = 10;
                while (Math.abs(tankDrive.Lift1.getCurrentPosition() - targ) > errMarg && Math.abs(tankDrive.Lift1.getCurrentPosition() - targ) > errMarg && opModeIsActive()) {
                    int error = targ - Math.max(tankDrive.Lift1.getCurrentPosition(), tankDrive.Lift2.getCurrentPosition());
                    double porg = Math.abs(error) * kp;
                    porg = Math.max(porg, powerFloor);
                    if (error < 0)
                        porg *= -1;
                    tankDrive.Lift1.setPower(porg);
                    tankDrive.Lift2.setPower(porg);

                    telemetry.addData("Porg:", porg);
                    telemetry.addData("Lift 1:", tankDrive.Lift1.getCurrentPosition());
                    telemetry.addData("Lift 2:", tankDrive.Lift2.getCurrentPosition());
                    telemetry.update();
                }
            } else if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                tankDrive.Lift1.setPower(gamepad2.left_stick_y / 2);
                tankDrive.Lift2.setPower(gamepad2.left_stick_y / 2);
            }
            telemetry.addData("Lift 1:", tankDrive.Lift1.getCurrentPosition());
            telemetry.addData("Lift 2:", tankDrive.Lift2.getCurrentPosition());
            telemetry.update();
        }

        //freeze();

    }
}
