package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;

@TeleOp(name="ModuleGoForward", group="Pushbot")
//@Disabled
public class ModuleGoForward extends LinearOpMode {

    ModuleHarware dsModule = new ModuleHarware();
    ElapsedTime time = new ElapsedTime();
    ArrayList<Double> timeRecord = new ArrayList<>();
    double targetAngle = 0;

    PIDController PID = new PIDController(0, 0, 0);
    public final double WHEEL_DIAMETER = 4; //Wheel diameter in inches
    public final int MOTOR_GEAR_TEETH = 40; //# of teeth on the motor gear
    public final int SMALL_BEVEL_GEAR_TEETH = 10; //# of teeth on the small bevel gear
    public final double GEAR_RATIO = (MOTOR_GEAR_TEETH + 0.0) / SMALL_BEVEL_GEAR_TEETH; //For every full turn of the motor, the wheel turns this many rotations.
    public final double MOTOR_TO_INCHES = GEAR_RATIO * WHEEL_DIAMETER * Math.PI; //For every full turn of both motors, the wheel moves forward this many inches
    public final int NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION = 1440;



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
            double jsaCache = joystickAngle();

            if (gamepad1.a) {
                //linearMovement(1, 3);
                idle();
            } else if (gamepad1.b) {
                moveOneModule(1);
            } else {
                telemetry.addData("Stop, ", "da** it!!");
                telemetry.update();
                dsModule.M0.setPower(0);
                dsModule.M1.setPower(0);
            }




        }

        telemetry.update();

    }

    public void moveOneModule(double inches)
    {
        resetEncoders();
        double encoderTicks = inches / MOTOR_TO_INCHES * NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION; // target number of encoder ticks
        double kp = 0.5 / 300;
        double errorMargin = 5; //Amount of error we are willing to accept in encoder ticks
        double powerFloor = 0.2;
        double powerCeiling = 0.4;
        telemetry.addData("Code is ", "running");
        telemetry.addData("encoderAvg", encoderAvg());
        telemetry.addData("encoderTicks", encoderTicks);
        telemetry.update();
        while (Math.abs(encoderAvg() - encoderTicks) > errorMargin )
        {
            double forwardpower = Math.abs(encoderTicks - encoderAvg()) * kp;
            forwardpower = Math.max(forwardpower, powerFloor);
            forwardpower = Math.min(forwardpower, powerCeiling);
            if (encoderTicks - encoderAvg() < 0)
                forwardpower *= -1;


            double targetAngle = 0;
            double headingError = targetAngle - currentAngle();
            double forwardIndex = forwardpower;
            if (headingError > 180)
                headingError = headingError - 360;
            if (headingError < -180)
                headingError = headingError + 360;
            double turnpower = 0.7 * (Math.abs(headingError / 180));

            telemetry.addData("Current position (in)", encoderAvg() * MOTOR_TO_INCHES / NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION);
            telemetry.addData("heading error", headingError);
            telemetry.addData("target angle", targetAngle);
            telemetry.addData("current angle", currentAngle());
            telemetry.addData("encoderAvg", encoderAvg());
            telemetry.addData("encoderTicks", encoderTicks);
            telemetry.addData("forwardPower", forwardpower);

            if (turnpower < 0.2)
                turnpower = 0.2;
            if (Math.abs(headingError) > 2)
            {
                if(headingError > 0)
                {
                    turnpower *= -1;
                }
            }
            else
            {
                turnpower = 0;
            }

            telemetry.addData("M0 Power: ",turnpower + forwardIndex);
            telemetry.addData("M1 Power: ",turnpower - forwardIndex);
            telemetry.update();

            dsModule.M0.setPower(turnpower + forwardIndex);
            dsModule.M1.setPower(turnpower - forwardIndex);
        }
    }

    /*
    public void moveDistance(double inches) {
        //Move module to direction 0 with existing code>

        double encoderTicks = inches / MOTOR_TO_INCHES * NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION; // target number of encoder ticks
        double kp = 0.5;
        double errorMargin = 0.1; //Amount of error we are willing to accept
        double powerFloor = 0.2;
        double powerCeiling = 1.0;
        while (leftModule.getAverageEncoder() < encoderTicks || rightModule.getAverageEncoder() < encoderTicks)
        {
            if (Math.abs(encoderTicks - leftModule.getAverageEncoder()) <= errorMargin)
            {
                double leftpower = (encoderTicks - leftModule.getAverageEncoder()) * kp;
                leftpower = Math.max(leftpower, powerFloor);
                leftpower = Math.min(leftpower, powerCeiling);
			<One Iteration of LinearMovement(leftpower)>
            }
            else
            {
                leftM1.setPower(0);
                leftM2.setPower(0); 		}

            if (Math.abs(encoderTicks - rightModule.getAverageEncoder()) <= errorMargin)
            {
                double rightpower = (encoderTicks - rightModule.getAverageEncoder()) * kp;
                rightpower = Math.max(rightpower, powerFloor);
                rightpower = Math.min(rightpower, powerCeiling);
			<One Iteration of LinearMovement(rightpower)>
            }
            else
            {
                leftM1.setPower(0);
                leftM2.setPower(0); 		}
        }
    }*/

    private void runWheel() {
        double targetAngle = joystickAngle();
        double error = targetAngle - currentAngle();
        double forwardIndex = gamepad1.right_stick_y * .5;
        if (error > 180)
            error = error - 360;
        if (error < -180)
            error = error + 360;
        double turnpower = 0.7 * (Math.abs(error / 180));

        telemetry.addData("error", error);
        telemetry.addData("target angle", targetAngle);
        telemetry.addData("current angle", currentAngle());
        telemetry.update();

        if (turnpower < 0.2)
            turnpower = 0.2;
        if (Math.abs(error) > 2)
        {
            if(error > 0)
            {
                turnpower *= -1;
            }
        }
        else
        {
            turnpower = 0;
        }

        dsModule.M0.setPower(turnpower + forwardIndex);
        dsModule.M1.setPower(turnpower - forwardIndex);

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
        double power = 0.7 * (Math.abs(error / 180));

        telemetry.addData("error", error);
        telemetry.addData("target angle", targetAngle);
        telemetry.addData("current angle", currentAngle());
        telemetry.update();

        if (power < 0.2)
            power = 0.2;
        if (Math.abs(error) > 2)
        {
            if(error > 0)
            {
                dsModule.M0.setPower(-power);
                dsModule.M1.setPower(-power);
            }
            else
            {
                dsModule.M0.setPower(power);
                dsModule.M1.setPower(power);
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

    public double currentAngle()
    {
        double rawAngle = (dsModule.encoder.getVoltage() - 0.047) * 72; //angle from 0 to 360
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

    public void resetEncoders()
    {
        dsModule.M0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dsModule.M1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
