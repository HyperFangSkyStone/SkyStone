package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="Separate Control Test", group="Monkey")
//@Disabled
public class ModuleTeleOp extends LinearOpMode {

    ModuleHarware dsModule = new ModuleHarware();
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
            if (gamepad1.dpad_up)
                turnWheel(0);
            else if (gamepad1.dpad_down)
                turnWheel(180);
            else if (gamepad1.dpad_left)
                turnWheel(-90);
            else if (gamepad1.dpad_right)
                turnWheel(90);
            else if (gamepad1.right_bumper)
                linearMovement(1, 0.5);
            //else if (gamepad1.left_bumper)
                //linearMovement(-1, 1);
            else if(gamepad1.y)
            {
                telemetry.addData("current angle", currentAngle());
                telemetry.update();
            }
            else {
                dsModule.M0.setPower(0);
                dsModule.M1.setPower(0);
            }
        }

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

    public void turnWheel(double targetAngle) {
        double error = targetAngle - currentAngle();
        if (error > 180)
            error = error - 360;
        if (error < -180)
            error = error + 360;
        double power = 0.7 * (Math.abs(error / 180));

        telemetry.addData("error", error);
        telemetry.addData("target angle", targetAngle);
        telemetry.addData("current angle", currentAngle());
        telemetry.update();

        if (power < 0.15)
            power = 0.15;
        if (Math.abs(error) > 2) {
            if (error > 0) {
                dsModule.M0.setPower(-power);
                dsModule.M1.setPower(-power);
            } else {
                dsModule.M0.setPower(power);
                dsModule.M1.setPower(power);
            }
        }
    }

    public double currentAngle()
    {
        double rawAngle = (dsModule.encoder.getVoltage()) * 72  - 32; //angle from 0 to 360

        return rawAngle;
    }






}
