package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;


@TeleOp(name="DSDrive TeleOp", group="Pushbot")
//@Disabled
public class DriveTrainTeleOp extends LinearOpMode {

    DriveTrainHardware dsDrive = new DriveTrainHardware();
    ElapsedTime time = new ElapsedTime();
    ArrayList<Double> timeRecord = new ArrayList<>();
    //target angle =


    PIDController PID = new PIDController(0, 0, 0);


    @Override
    public void runOpMode() throws InterruptedException
    {
        dsDrive.init(hardwareMap);

        waitForStart();


        while(opModeIsActive())
        {
            linearMovement(0.8, 'l');
        }

    }



    //  ++++++ Helper Methods ++++++

    public void linearMovement(double inputPower, char m)
    {
        double decay = 0.99;

        if (joystickAngle() > -10 && joystickAngle() < 10)
        {
            if(motor(m,1).getCurrentPosition() > encoderAvg(m))
            {
                motor(m, 1).setPower(inputPower * decay);
                motor(m, 2).setPower(inputPower);
            }
            else if (motor(m,2).getCurrentPosition() > encoderAvg(m))
            {
                motor(m, 2).setPower(inputPower * decay);
                motor(m, 1).setPower(inputPower);
            }
            else
            {
                motor(m, 1).setPower(inputPower);
                motor(m, 2).setPower(inputPower);
            }
        }
        else
        {
            motor(m, 1).setPower(0);
            motor(m, 2).setPower(0);
        }


    }

    public void turnWheel(double angle)
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
        return angle;
    }

    public double encoderAvg(char module)
    {
        /*
            parameter:
            'l' - calculate leftModule
            'r' - calculate rightModule

         */

        return (Math.abs(motor(module, 1).getCurrentPosition()) + Math.abs(motor(module, 2).getCurrentPosition())) / 2;
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

    /*public double getWheelHeading(char module)
    {
        if (module == 'l')
            return (dsDrive.LeftEncoder.getVoltage() /5 * 360);
    }*/

}
