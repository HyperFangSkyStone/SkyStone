package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
public final double WHEEL_DIAMETER = 90; //Wheel diameter in mm
public final int MOTOR_GEAR_TEETH = 26; //# of teeth on the motor gear
public final int WHEEL_GEAR_TEETH = 20; //# of teeth on the wheel gear
public final double GEAR_RATIO = (MOTOR_GEAR_TEETH + 0.0) / WHEEL_GEAR_TEETH; //For every full turn of the motor, the wheel turns this many rotations.
public final double MM_TO_INCHES =  25.4;
public final double MOTOR_TO_INCHES = GEAR_RATIO * WHEEL_DIAMETER * Math.PI / MM_TO_INCHES; //For every full turn of both motors, the wheel moves forward this many inches


@TeleOp(name="TankDrivePIDTeleop", group="Sans Sans")
//@Disabled
public class TankDriveTeleop extends LinearOpMode {

    TankDriveHardware dsModule = new TankDriveHardware();
    ElapsedTime time = new ElapsedTime();
    ArrayList<Double> timeRecord = new ArrayList<>();


    PIDController PID = new PIDController(0, 0, 0);


    @Override
    public void runOpMode() throws InterruptedException {
        dsModule.init(hardwareMap);

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                movethForward(10,10);
            } else if (gamepad1.dpad_right)
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

    public void linearMovement(double inputPower, double t) {

        time.reset();

        while (time.seconds() < t) {
            dsModule.LM0.setPower(inputPower);
            dsModule.LM1.setPower(inputPower);
            dsModule.RM0.setPower(inputPower);
            dsModule.RM1.setPower(inputPower);
            telemetry.update();
        }
        dsModule.M0.setPower(0);
        dsModule.M1.setPower(0);
    }

    public void movethForward(double inches, double t)
    {
        time.reset();
        double encoderTicks = inches / MOTOR_TO_INCHES * NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION; // target number of encoder ticks
        double kp = 0.005;
        int errorMargin = 20; //Amount of error in ticks we are willing to accept
        double powerFloor = 0.3; //Minimum power
        double powerCeiling = 0.8; //Maximum power
        while ((dsModule.getAverageEncoder('l') < encoderTicks || dsModule.getAverageEncoder('r') < encoderTicks) && time.seconds() < t)
        {
            if (Math.abs(encoderTicks - dsModule.getAverageEncoder('l')) <= errorMargin)
            {
                double leftpower = Math.abs(encoderTicks - dsModule.getAverageEncoder('l')) * kp;
                leftpower = Math.max(leftpower, powerFloor);
                leftpower = Math.min(leftpower, powerCeiling);
                if (encoderTicks - dsModule.getAverageEncoder('l') < 0) leftpower *= -1;
                dsModule.LM0.setPower(leftpower);
                dsModule.LM1.setPower(leftpower);
            }
            else {
                dsModule.LM0.setPower(0);
                dsModule.LM1.setPower(0);
            }

            if (Math.abs(encoderTicks - dsModule.getAverageEncoder('r')) <= errorMargin)
            {
                double rightpower = Math.abs(encoderTicks - dsModule.getAverageEncoder('r')) * kp;
                rightpower = Math.max(rightpower, powerFloor);
                rightpower = Math.min(rightpower, powerCeiling);
                if (encoderTicks - dsModule.getAverageEncoder('r') < 0) rightpower *= -1;
                dsModule.RM0.setPower(rightpower);
                dsModule.RM1.setPower(rightpower);
            }
            else {
                dsModule.RM0.setPower(0);
                dsModule.RM1.setPower(0);
            }
        }
    }
}
