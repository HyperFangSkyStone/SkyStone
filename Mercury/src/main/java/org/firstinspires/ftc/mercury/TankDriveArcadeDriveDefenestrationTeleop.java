package org.firstinspires.ftc.mercury;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="Arcade", group="1")
@Disabled
public class TankDriveArcadeDriveDefenestrationTeleop extends LinearOpMode {

    TankDriveALPHA tankDrive = new TankDriveALPHA();
    ElapsedTime clock = new ElapsedTime();
    ArrayList<Double> timeRecord = new ArrayList<>();


    PIDController PID = new PIDController(0, 0, 0);


    @Override
    public void runOpMode() throws InterruptedException
    {
        tankDrive.init(hardwareMap);

        waitForStart();


        while(opModeIsActive())
        {
            /*if (gamepad1.left_bumper)
            {
                double forwardPower = -gamepad1.left_stick_y;
                double turnPower = gamepad1.left_stick_x;
                tankDrive.LM0.setPower(forwardPower + turnPower);
                tankDrive.LM1.setPower(forwardPower + turnPower);
                tankDrive.RM0.setPower(forwardPower - turnPower);
                tankDrive.RM1.setPower(forwardPower - turnPower);
            }*/
            /*if (Math.abs(gamepad1.left_stick_y) > 0.2)
            {
                tankDrive.LM0.setPower(-gamepad1.left_stick_y);
                tankDrive.LM1.setPower(-gamepad1.left_stick_y);
            }
            else
            {
                tankDrive.LM0.setPower(0);
                tankDrive.LM1.setPower(0);
            }

            if (Math.abs(gamepad1.right_stick_y) > 0.2)
            {
                tankDrive.RM0.setPower(-gamepad1.right_stick_y);
                tankDrive.RM1.setPower(-gamepad1.right_stick_y);
            }
            else
            {
                tankDrive.RM0.setPower(0);
                tankDrive.RM1.setPower(0);
            }*/

            if (Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2)
            {
                double forwardPower = -gamepad1.left_stick_y;
                double turnPower = gamepad1.right_stick_x;
                tankDrive.LM0.setPower(forwardPower + turnPower);
                tankDrive.LM1.setPower(forwardPower + turnPower);
                tankDrive.RM0.setPower(forwardPower - turnPower);
                tankDrive.RM1.setPower(forwardPower - turnPower);
            }
            else
            {
                freeze();
            }


            if (Math.abs(gamepad2.left_stick_y) > 0.2)
            {
                if(!gamepad2.right_bumper)
                    tankDrive.Intake1.setPower(-gamepad2.left_stick_y);
                else
                {
                    tankDrive.Intake1.setPower(-gamepad2.left_stick_y / 2);
                    telemetry.addData("Intake1 is at", " half speed");
                }
            }
            else
            {
                tankDrive.Intake1.setPower(0);
            }

            if (Math.abs(gamepad2.right_stick_y) > 0.2)
            {
                if(!gamepad2.right_bumper)
                    tankDrive.Intake2.setPower(-gamepad2.right_stick_y);
                else
                {
                    tankDrive.Intake2.setPower(-gamepad2.right_stick_y / 2);
                    telemetry.addData("Intake2 is at", " half speed");
                }
            }
            else
            {
                tankDrive.Intake2.setPower(0);
            }

            if(gamepad2.dpad_up)
            {
                claw(false);
            }
            else if (gamepad2.dpad_down)
            {
                claw(true);
            }
        }

        freeze();

    }




    //  ++++++ Helper Methods ++++++

    public void linearMovement(double inputPower, double t)
    {

        clock.reset();

        while (clock.seconds() < t && opModeIsActive())
        {
            tankDrive.LM0.setPower(inputPower);
            tankDrive.LM1.setPower(inputPower);
            tankDrive.RM0.setPower(inputPower);
            tankDrive.RM1.setPower(inputPower);
            telemetry.update();
        }
        tankDrive.LM0.setPower(0);
        tankDrive.LM1.setPower(0);
        tankDrive.RM0.setPower(0);
        tankDrive.RM1.setPower(0);
    }

    public void pidLinearMovement(double distance)
    {
        double conversionIndex = 537.6/14.47111;
        double timeFrame = 5; //distance * distanceTimeIndex;
        double errorMargin = 20;
        double kP = 0.002;
        double powerFloor = 0.2;
        double powerCeiling = 0.8;
        double targetTick = distance * conversionIndex;
        double output;
        clock.reset();
        tankDrive.resetEncoders();
        while (clock.seconds() < timeFrame && Math.abs(tankDrive.getEncoderAvg() - targetTick) > errorMargin  && opModeIsActive())
        {
            //output = linearPID.PIDOutput(targetTick,averageEncoderTick(),clock.seconds());
            output = Math.abs(targetTick - tankDrive.getEncoderAvg()) * kP;
            output = Math.max(output, powerFloor);
            output = Math.min(output, powerCeiling);
            if (targetTick - tankDrive.getEncoderAvg() < 0) output *= -1;
            runMotor(output, -output);
        }
        runMotor(0,0);
    }

    public void turnToAngle(double angle)
    {

    }

    public double averageEncoderTick()
    {
        double output = tankDrive.LM0.getCurrentPosition() + tankDrive.LM1.getCurrentPosition() + tankDrive.RM0.getCurrentPosition() +tankDrive.RM0.getCurrentPosition();
        output /= 4.0;
        return output;
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

    public void runMotor(double leftInput, double rightInput)
    {
        tankDrive.LM0.setPower(leftInput);
        tankDrive.LM1.setPower(leftInput);
        tankDrive.RM0.setPower(rightInput);
        tankDrive.RM1.setPower(rightInput);
    }
    public void freeze()
    {
        tankDrive.LM0.setPower(0);
        tankDrive.LM1.setPower(0);
        tankDrive.RM0.setPower(0);
        tankDrive.RM1.setPower(0);
    }
    public void claw(boolean x) {
        if (x) {
            tankDrive.LeftFang.setPosition(1); //true = up
            tankDrive.RightFang.setPosition(0);
        } else {
            tankDrive.LeftFang.setPosition(0); //false = down
            tankDrive.RightFang.setPosition(1);
        }
    }
}
