package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="ModuleTest with absolute encoder", group="Pushbot")
//@Disabled
public class ModuleTest extends LinearOpMode {

    ModuleHarware dsModule = new ModuleHarware();
    ElapsedTime time = new ElapsedTime();
    ArrayList<Double> timeRecord = new ArrayList<>();
    double targetAngle = 0;

    PIDController PID = new PIDController(0, 0, 0);


    @Override
    public void runOpMode() throws InterruptedException
    {
        dsModule.init(hardwareMap);

        waitForStart();


        while(opModeIsActive())
        {
            //targetAngle = joystickAngle();
            //telemetry.addData("encoderAngle", currentAngle());
            //telemetry.addData("targetAngle", targetAngle);
            //double jsaCache = joystickAngle();

            if (gamepad1.a)
            {
                //linearMovement(1, 3);
                idle();
            }
            if (gamepad1.b)
            {
                turnWheel();
            }
            else
            {
                dsModule.M0.setPower(0);
                dsModule.M1.setPower(0);
            }


            if(gamepad1.y)
            {
                telemetry.addData("current angle", currentAngle());
                telemetry.update();

            }




        }

        telemetry.update();

    }

    private void runWheel() {
        double targetAngle = joystickAngle();
        double error = targetAngle - currentAngle();
        double forwardIndex = gamepad1.left_stick_y * .5;
        if (error > 180)
            error = error - 360;
        if (error < -180)
            error = error + 360;
        double turnpower = 0.7 * (Math.abs(error / 180));
    }


    //  ++++++ Helper Methods ++++++

    public void linearMovement(double inputPower, double t)
    {
        double p = 0.99;

        time.reset();
        double targetAngle = currentAngle();
        double error;

        while (time.seconds() < t)
        {
            error = targetAngle - currentAngle();
            if (error > 180)
                error = error - 360;
            if (error < -180)
                error = error + 360;
            p = 1 - (0.05 * Math.abs(error));
            if (p < 0.8)
                p = 0.8;

            telemetry.addData("error", error);
            telemetry.addData("target angle", targetAngle);
            telemetry.addData("current angle", currentAngle());
            telemetry.addData("correction index", p);


            if(Math.abs(error) < 1) //if error is within 1 degree
            {
                dsModule.M0.setPower(inputPower); //no proportion
                dsModule.M1.setPower(-inputPower);
                telemetry.addData("correct", inputPower);
            }
            else //if error is greater than 1 degree
            {
                if (error < 0)
                {
                    dsModule.M0.setPower(inputPower);
                    dsModule.M1.setPower(-inputPower * p);
                    telemetry.addData("M0", inputPower);
                    telemetry.addData("M1", inputPower * p);
                }
                else
                {
                    dsModule.M0.setPower(inputPower * p);
                    dsModule.M1.setPower(-inputPower);
                    telemetry.addData("M0", inputPower * p);
                    telemetry.addData("M1", inputPower);
                }
            }
            telemetry.update();
        }
        dsModule.M0.setPower(0);
        dsModule.M1.setPower(0);
    }

    public void turnWheel()
    {
        double targetAngle = joystickAngle();
        double error = targetAngle - currentAngle();
        if (error > 180)
            error = error - 360;
        if (error < -180)
            error = error + 360;
        double power = 0.2 * (Math.abs(error / 180));

        if (power < 0.15)
            power = 0.15;
        if (Math.abs(error) > 5)
        {
            if(error > 0)
            {
                dsModule.M0.setPower(power);
                dsModule.M1.setPower(power);
            }
            else
            {
                dsModule.M0.setPower(-power);
                dsModule.M1.setPower(-power);
            }


        }
        else
        {
            dsModule.M0.setPower(0);
            dsModule.M1.setPower(0);
        }

    }

    public void turnToAngle(double TargetAngle)
    {
        
    }



    public double joystickAngle()
    {
        double angle;
        double x = gamepad1.right_stick_x;
        double y = -gamepad1.right_stick_y;

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

    public double currentAngle()
    {
        double rawAngle = (dsModule.encoder.getVoltage()) * 72  - 16.2; //angle from 0 to 360
        double outputAngle;

        if (rawAngle < 180)
            outputAngle = - rawAngle;
        else
            outputAngle = 360 - rawAngle;
        return rawAngle;
    }

    public double encoderAvg()
    {
        /*
            parameter:
            'l' - calculate leftModule
            'r' - calculate rightModule

         */

        return (Math.abs(dsModule.M0.getCurrentPosition()) + Math.abs(dsModule.M1.getCurrentPosition())) / 2;
    }

}
