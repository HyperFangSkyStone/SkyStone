package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="TankDriveTest", group="Sans Sans")
//@Disabled
public class TankDriveTeleop extends LinearOpMode {

    TankDriveHardware dsModule = new TankDriveHardware();
    ElapsedTime time = new ElapsedTime();
    ArrayList<Double> timeRecord = new ArrayList<>();


    PIDController PID = new PIDController(0, 0, 0);


    @Override
    public void runOpMode() throws InterruptedException
    {
        dsModule.init(hardwareMap);

        waitForStart();


        while(opModeIsActive())
        {
            if (gamepad1.left_bumper)
            {
                double forwardPower = -gamepad.left_stick_y;
                double turnPower = gamepad.left_stick_x;
                dsModule.LM0.setPower(left_stick_y + left_stick_x);
                dsModule.LM1.setPower(left_stick_y + left_stick_x);
                dsModule.RM0.setPower(left_stick_y - left_stick_x);
                dsModule.RM1.setPower(left_stick_y - left_stick_x);
            }
            else if (gamepad1.dpad_right)
                linearMovement(0.5, 0.5);
            else if (gamepad1.right_bumper)
                linearMovement(1, 0.5);
                //else if (gamepad1.left_bumper)
                //linearMovement(-1, 1);
            else {
                dsModule.LM0.setPower(0);
                dsModule.LM1.setPower(0);
                dsModule.RM0.setPower(0);
                dsModule.RM1.setPower(0);
            }
        }

    }




    //  ++++++ Helper Methods ++++++

    public void linearMovement(double inputPower, double t)
    {

        time.reset();

        while (time.seconds() < t)
        {
            dsModule.LM0.setPower(inputPower);
            dsModule.LM1.setPower(inputPower);
            dsModule.RM0.setPower(inputPower);
            dsModule.RM1.setPower(inputPower);
            telemetry.update();
        }
        dsModule.M0.setPower(0);
        dsModule.M1.setPower(0);
    }

    public double joystickAngle()
    {
        double angle;
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        if ((y == 1 && x == 0) || (x == 0 && y == 0))
            angle = 0;
        else if (y == -1 && x == 0)
            angle = 180;
        else if (y == 0 && x == 1)
            angle = 90;
        else if (y == 0 && x == -1)
            angle = -90;
        else if (x > 0 && y > 0)
            angle = Math.atan(x/y) * 180 / Math.PI;
        else if (x < 0 && y > 0)
            angle = Math.atan(x/y) * 180 / Math.PI;
        else if (x < 0 && y < 0)
            angle = Math.atan(x/y) * 180 / Math.PI - 180;
        else
            angle = Math.atan(x/y) * 180 / Math.PI + 180;
        return -angle;
    }
}
