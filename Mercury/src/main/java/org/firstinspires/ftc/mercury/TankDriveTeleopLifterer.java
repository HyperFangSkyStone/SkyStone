package org.firstinspires.ftc.mercury;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// 85 ticks / inch

@TeleOp(name="Tank Lifterer", group="1")
@Disabled
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
                double powerFloor = 0.35;
                int errMarg = 10;
                while (Math.abs(tankDrive.Lift1.getCurrentPosition() - targ) > errMarg && Math.abs(tankDrive.Lift1.getCurrentPosition() - targ) > errMarg && opModeIsActive()) {
                    int error = targ - Math.max(Math.abs(tankDrive.Lift1.getCurrentPosition()), Math.abs(tankDrive.Lift2.getCurrentPosition()));
                    double porg = Math.abs(error) * kp;
                    porg = Math.max(porg, powerFloor);
                    if (error < 0)
                        porg *= -1;
                    tankDrive.Lift1.setPower(porg);
                    tankDrive.Lift2.setPower(porg);

                    telemetry.addData("Porg:", porg);
                    telemetry.addData("Lift 1:", tankDrive.Lift1.getCurrentPosition());
                    telemetry.addData("Lift 2:", tankDrive.Lift2.getCurrentPosition());
                    telemetry.addData("Error:", error);
                    telemetry.update();
                }
                tankDrive.Lift1.setPower(0);
                tankDrive.Lift2.setPower(0);
            } else if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                tankDrive.Lift1.setPower(gamepad2.left_stick_y / 2);
                tankDrive.Lift2.setPower(gamepad2.left_stick_y / 2);
            } else {
                tankDrive.Lift1.setPower(0.0);
                tankDrive.Lift2.setPower(0.0);
            }
            telemetry.addData("Lift 1:", tankDrive.Lift1.getCurrentPosition());
            telemetry.addData("Lift 2:", tankDrive.Lift2.getCurrentPosition());
            telemetry.update();
        }

        //freeze();

    }
}
