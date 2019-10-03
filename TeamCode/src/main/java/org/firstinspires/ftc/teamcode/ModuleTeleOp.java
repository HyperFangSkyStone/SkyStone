package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="Module Test", group="Pushbot")
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
            if (Math.abs(joystickAngle()) < 10) {
                linearMovement(0.8);
            }
            else {
                dsModule.M0.setPower(0);
                dsModule.M1.setPower(0);
            }

            telemetry.addData("M0", dsModule.M0.getCurrentPosition());
            telemetry.addData("M1", dsModule.M1.getCurrentPosition());
            if (dsModule.M0.getCurrentPosition() > dsModule.M1.getCurrentPosition())
                telemetry.addData("", "M0");
            else if (dsModule.M0.getCurrentPosition() < dsModule.M1.getCurrentPosition())
                telemetry.addData("", "M1");
            else
                telemetry.addData("", "equal");
        }

        telemetry.update();

    }




    //  ++++++ Helper Methods ++++++

    public void linearMovement(double inputPower)
    {
        double decay = 0.99;

        if (joystickAngle() > -10 && joystickAngle() < 10)
        {
            if(dsModule.M0.getCurrentPosition() > encoderAvg())
            {
                dsModule.M0.setPower(inputPower * decay);
                dsModule.M1.setPower(-inputPower);
            }
            else if (dsModule.M1.getCurrentPosition() > encoderAvg())
            {
                dsModule.M0.setPower(inputPower);
                dsModule.M1.setPower(-inputPower * decay);
            }
            else
            {
                dsModule.M0.setPower(inputPower);
                dsModule.M1.setPower(-inputPower);
            }
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
