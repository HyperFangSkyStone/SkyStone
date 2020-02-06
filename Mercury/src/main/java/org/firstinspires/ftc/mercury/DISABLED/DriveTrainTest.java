package org.firstinspires.ftc.mercury.DISABLED;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.mercury.DISABLED.DSDriveTrainHardware;
import org.firstinspires.ftc.mercury.PIDController;

import java.util.ArrayList;

@TeleOp(name="DSDriveTrainTestSEP", group="Pushbot") //diffyswerve
@Disabled
public class DriveTrainTest extends LinearOpMode {

    DSDriveTrainHardware dsDrive = new DSDriveTrainHardware();
    ElapsedTime time = new ElapsedTime();
    ArrayList<Double> timeRecord = new ArrayList<>();


    double leftEncOffset = 0;
    double rightEncOffset = 0;

    PIDController PID = new PIDController(0, 0, 0);


    @Override
    public void runOpMode() throws InterruptedException
    {
        dsDrive.init(hardwareMap);

        waitForStart();
        leftEncOffset = 0;
        rightEncOffset = 0;
        tuneEncoders();
        while(opModeIsActive())
        {
            if (gamepad1.y)
            {
                telemetry.addData("Left Angle", currentAngle('l'));
                telemetry.addData("Right Angle", currentAngle('r'));
                telemetry.update();
            }
            /*else if (gamepad1.b)
            {
                motor('l', 1).setDirection(DcMotor.Direction.REVERSE);
                motor('l', 2).setDirection(DcMotor.Direction.REVERSE);
                motor('r', 1).setDirection(DcMotor.Direction.REVERSE);
                motor('r', 1).setDirection(DcMotor.Direction.REVERSE);
            }*/
            else if (gamepad1.right_bumper) {
                doubleLinearMovement(0.5);
            }
            else if (gamepad1.left_bumper) {
                doubleLinearMovement(1);
            }
            else {
                motor('l', 1).setPower(0);
                motor('l', 2).setPower(0);
                motor('r', 1).setPower(0);
                motor('r', 2).setPower(0);
            }
        }


    }



    //  ++++++ Helper Methods ++++++


    public void correctToAngle(double targetAngle)
    {

    }

    public void turnWheel(double targetAngle, char module) {
        double error = targetAngle - currentAngle(module);
        if (error > 180)
            error = error - 360;
        if (error < -180)
            error = error + 360;
        double power = 0.7 * (Math.abs(error / 180));

        if (power < 0.15)
            power = 0.15;
        if (Math.abs(error) > 2) {
            if (error > 0) {
                motor(module, 1).setPower(-power);
                motor(module, 2).setPower(-power);
            } else {
                motor(module, 1).setPower(power);
                motor(module, 2).setPower(power);
            }
        }
    }


    public double joystickAngle(char lr)
    {
        double angle, x, y;
        if (lr == 'l') {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
        }
        else
        {
            x = gamepad1.right_stick_x;
            y = -gamepad1.right_stick_y;
        }

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
        return angle;
    }

    public void linearMovement(double inputPower, double t, char module)
    {
        double p = 0.99;

        time.reset();
        double targetAngle = currentAngle(module);
        double error;

        while (time.seconds() < t)
        {
            error = targetAngle - currentAngle(module);
            if (error > 180)
                error = error - 360;
            if (error < -180)
                error = error + 360;
            p = 1 - (0.05 * Math.abs(error));
            if (p < 0.8)
                p = 0.8;

            telemetry.addData("error", error);
            telemetry.addData("target angle", targetAngle);
            telemetry.addData("current angle", currentAngle(module));
            telemetry.addData("correction index", p);


            if(Math.abs(error) < 1) //if error is within 1 degree
            {
                motor(module, 1).setPower(inputPower); //no proportion
                motor(module, 2).setPower(-inputPower);
                telemetry.addData("correct", inputPower);
            }
            else //if error is greater than 1 degree
            {
                if (error < 0)
                {
                    motor(module, 1).setPower(inputPower);
                    motor(module, 2).setPower(-inputPower * p);
                    telemetry.addData("M0", inputPower);
                    telemetry.addData("M1", inputPower * p);
                }
                else
                {
                    motor(module, 1).setPower(inputPower * p);
                    motor(module, 2).setPower(-inputPower);
                    telemetry.addData("M0", inputPower * p);
                    telemetry.addData("M1", inputPower);
                }
            }
            telemetry.update();
        }
        motor(module, 1).setPower(0);
        motor(module, 2).setPower(0);
    }


    public double currentAngle(char lr)
    {
        double rawAngle = 420;
        if (lr == 'l')
            rawAngle = (dsDrive.LeftEncoder.getVoltage()) * 72 - leftEncOffset; //angle from 0 to 360
        if (lr == 'r')
            rawAngle = (dsDrive.RightEncoder.getVoltage()) * 72 - rightEncOffset;


        return rawAngle;
    }




    public DcMotor motor(char module, int motor)
    {
        /*
            parameter:
            l/r - left/right
            1/2 - motor 1/2
         */

        if (module == 'l')
        {
            if (motor == 1)
                return dsDrive.LeftM1;
            else
                return dsDrive.LeftM2;
        }
        else
        {
            if (motor == 1)
                return dsDrive.RightM1;
            else
                return dsDrive.RightM2;
        }
    }

    public void tuneEncoders() //allows the encoders to be tuned upon startup
    {
        if(Math.abs(0 - (currentAngle('l') + currentAngle('r'))/2) < 90 ) //if resetting to 0 degrees
        {
            leftEncOffset = currentAngle('l');
            rightEncOffset = currentAngle('r');
        }
        else
        {
            leftEncOffset = currentAngle('l') - 180;
            rightEncOffset = currentAngle('r') - 180;
        }
    }


    public void doubleLinearMovement(double inputPower)
    {
        double pl, pr;

        time.reset();
        double targetAngle = 0;
        double errorLeft, errorRight;
        errorLeft = targetAngle - currentAngle('l');
        errorRight = targetAngle - currentAngle('r');

        if (errorLeft > 180)
            errorLeft -= 360;
        if (errorLeft < -180)
            errorLeft += 360;
        if (errorRight > 180)
            errorRight -= 360;
        if (errorRight < -180)
            errorRight += 360;

        pl = 1 - (0.05 * Math.abs(errorLeft));
        pr = 1 - (0.05 * Math.abs(errorRight));

        if (pl < 0.8)
            pl = 0.8;
        if (pr < 0.8)
            pr = 0.8;

        telemetry.addData("current left", currentAngle('l'));
        telemetry.addData("left error", errorLeft);
        telemetry.addData("current right", currentAngle('r'));
        telemetry.addData("right error", errorRight);
        telemetry.update();

        if(Math.abs(errorLeft) < 1)
        {
            motor('l', 1).setPower(inputPower); //no proportion
            motor('l', 2).setPower(-inputPower);
        }
        else //if error is greater than 1 degree
        {
            if (errorLeft < 0)
            {
                motor('l', 1).setPower(inputPower);
                motor('l', 2).setPower(-inputPower * pl);
            }
            else
            {
                motor('l', 1).setPower(inputPower * pl);
                motor('l', 2).setPower(-inputPower);
            }
        }


        if(Math.abs(errorRight) < 1)
        {
            motor('r', 1).setPower(inputPower); //no proportion
            motor('r', 2).setPower(-inputPower);
        }
        else //if error is greater than 1 degree
        {
            if (errorRight < 0)
            {
                motor('r', 1).setPower(inputPower);
                motor('r', 2).setPower(-inputPower * pr);
            }
            else
            {
                motor('r', 1).setPower(inputPower * pr);
                motor('r', 2).setPower(-inputPower);
            }
        }

    }

}