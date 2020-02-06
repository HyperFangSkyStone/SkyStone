package org.firstinspires.ftc.mercury.DISABLED;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.mercury.PIDController;
import org.firstinspires.ftc.mercury.TankDriveALPHA;

import java.util.ArrayList;

@Disabled
@TeleOp(name="Arc Test", group="2")
public class ArcTest extends LinearOpMode {

    TankDriveALPHA tankDrive = new TankDriveALPHA();
    ElapsedTime clock = new ElapsedTime();
    ArrayList<Double> timeRecord = new ArrayList<>();


    PIDController PID = new PIDController(0, 0, 0);


    boolean intakeOn = false;
    int intakeDir = 1;


    public final double WHEEL_DIAMETER = 90; //Wheel diameter in mm
    public final int MOTOR_GEAR_TEETH = 26; //# of teeth on the motor gear
    public final int WHEEL_GEAR_TEETH = 20; //# of teeth on the wheel gear
    public final double GEAR_RATIO = (MOTOR_GEAR_TEETH + 0.0) / WHEEL_GEAR_TEETH; //For every full turn of the motor, the wheel turns this many rotations.
    public final double MM_TO_INCHES =  25.4;
    public final double MOTOR_TO_INCHES = GEAR_RATIO * WHEEL_DIAMETER * Math.PI / MM_TO_INCHES; //For every full turn of both motors, the wheel moves forward this many inches
    public final double NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION = 537.6;

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
            if (Math.abs(gamepad1.left_stick_y) > 0.2)
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
            }

            /*
            else
            {
                tankDrive.LM0.setPower(0);
                tankDrive.LM1.setPower(0);
                tankDrive.RM0.setPower(0);
                tankDrive.RM1.setPower(0);
            }*/

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


            if (gamepad1.x)
            {
                double d1 = 50;
                double d2 = 50 - 7 * Math.PI;
                movethForward(d1, d2, 5, 0.6, 0.8);
            }

        }

        freeze();

    }




    //  ++++++ Helper Methods ++++++

    public void linearMovement(double inputPower, double t)
    {

        clock.reset();

        while (clock.seconds() < t)
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

    public void pidLinearMovement(double distance, double kd)
    {
        double conversionIndex = 537.6/((26.0/20.0)*90.0* Math.PI / 25.4); // Ticks per inch
        double timeFrame = 5; //distance * distanceTimeIndex;
        double errorMargin = 5;
        double powerFloor = 0.2;
        double powerCeiling = 0.8;

        clock.reset();
        tankDrive.resetEncoders();

        double targetTick = distance / MOTOR_TO_INCHES * NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION *50/47;
        telemetry.addData("ticks", targetTick);
        telemetry.update();
        double error = targetTick;
        double errorPrev = 0;
        double kP = 0.0005;
        double kD = kd;
        double p, d;
        double output;
        double time = clock.seconds();
        double timePrev = 0;


        while (clock.seconds() < timeFrame && Math.abs(error) > errorMargin && opModeIsActive())
        {
            //output = linearPID.PIDOutput(targetTick,averageEncoderTick(),clock.seconds());

            p = Math.abs(error) * kP;
            d = ((error - errorPrev) / (time - timePrev)) /targetTick * kD;

            output = p + d;
            output = Math.max(output, powerFloor);
            output = Math.min(output, powerCeiling);
            if (error < 0) output *= -1;
            runMotor(output, output);

            errorPrev = error;
            error = targetTick - tankDrive.getEncoderAvg(telemetry);

            timePrev = time;
            time = clock.seconds();

            telemetry.addData("Target", targetTick);
            telemetry.addData("Current", averageEncoderTick());
            telemetry.addData("RM0", tankDrive.RM0.getCurrentPosition());
            telemetry.addData("RM1", tankDrive.RM1.getCurrentPosition());
            telemetry.addData("LM0", tankDrive.LM0.getCurrentPosition());
            telemetry.addData("LM1", tankDrive.LM1.getCurrentPosition());
            telemetry.addData("error", error);
            telemetry.addData("kP", kP);
            telemetry.addData("output", output);
            telemetry.update();


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

    public void claw(boolean x) {/*
        if (x) {
            tankDrive.LServo.setPosition(1); //true = up
            tankDrive.RServo.setPosition(0);
        } else {
            tankDrive.LServo.setPosition(0); //false = down
            tankDrive.RServo.setPosition(1);
        }*/
    }


    public double sigmoid(double error, double ceiling, double floor, double half, double stiff) {
        return floor + (ceiling - floor) / (1 + Math.pow(Math.E, stiff * (half - error)));
    }

    public void movethForward(double inchesL, double inchesR, double t, double kp, double power)
    {
        tankDrive.resetEncoders();
        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        double encoderTicksL = inchesL / MOTOR_TO_INCHES * NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION; // target number of encoder tick
        double encoderTicksR = inchesR / MOTOR_TO_INCHES * NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION;
        int errorMargin = 5; //Amount of error in ticks we are willing to accept
        double powerFloor = 0.19; //Minimum power
        double powerCeiling = power; //Maximum power

        while (Math.abs(tankDrive.getAverageEncoder('l') + tankDrive.getAverageEncoder('r') - encoderTicksL * 2) > errorMargin && clock.seconds() < t)
        {
            if (Math.abs(encoderTicksL - tankDrive.getAverageEncoder('l')) >= errorMargin)
            {
                double leftpower = Math.abs((encoderTicksL - tankDrive.getAverageEncoder('l'))/encoderTicksL) * kp;
                //double leftpower = sigmoid(encoderTicksL - tankDrive.getAverageEncoder('l'), powerCeiling, powerFloor, 800, kp * 4);
                leftpower = Math.max(leftpower, powerFloor);
                leftpower = Math.min(leftpower, powerCeiling);
                if (encoderTicksL - tankDrive.getAverageEncoder('l') < 0) leftpower *= -1;
                tankDrive.LM0.setPower(leftpower);
                tankDrive.LM1.setPower(leftpower);
                telemetry.addData("LeftPower", leftpower);
            }
            else {
                tankDrive.LM0.setPower(0);
                tankDrive.LM1.setPower(0);
            }

            if (Math.abs(encoderTicksR - tankDrive.getAverageEncoder('r')) >= errorMargin)
            {
                double rightpower = Math.abs((encoderTicksR - tankDrive.getAverageEncoder('r'))/encoderTicksR) * kp;
                //double rightpower = sigmoid(encoderTicksR - tankDrive.getAverageEncoder('r'), powerCeiling, powerFloor, 800, kp * 6);
                rightpower = Math.max(rightpower, powerFloor);
                rightpower = Math.min(rightpower, powerCeiling);
                if (encoderTicksR - tankDrive.getAverageEncoder('r') < 0) rightpower *= -1;
                tankDrive.RM0.setPower(rightpower);
                tankDrive.RM1.setPower(rightpower);
                telemetry.addData("RightPower", rightpower);
            }
            else {
                tankDrive.RM0.setPower(0);
                tankDrive.RM1.setPower(0);
            }
            telemetry.addData("Left Encoder", tankDrive.getAverageEncoder('l'));
            telemetry.addData("Right Encoder", tankDrive.getAverageEncoder('r'));
            telemetry.addData("L", encoderTicksL);
            telemetry.addData("R", encoderTicksR);
            telemetry.addData("Current L", tankDrive.getAverageEncoder('l'));
            telemetry.addData("Current R", tankDrive.getAverageEncoder('l'));
            telemetry.addData("kP", kp);
            telemetry.update();

        }
    }
}
