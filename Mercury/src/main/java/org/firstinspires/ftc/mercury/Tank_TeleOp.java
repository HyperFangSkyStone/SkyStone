package org.firstinspires.ftc.mercury;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="ChimpanzeeAutoPIDTest", group="1")
@Disabled
public class Tank_TeleOp extends LinearOpMode {

    TankDriveALPHA tankDrive = new TankDriveALPHA();
    ElapsedTime clock = new ElapsedTime();
    ElapsedTime intakeClock = new ElapsedTime();
    ArrayList<Double> timeRecord = new ArrayList<>();


    PIDController PID = new PIDController(0, 0, 0);



    public final double WHEEL_DIAMETER = 90; //Wheel diameter in mm
    public final int MOTOR_GEAR_TEETH = 26; //# of teeth on the motor gear
    public final int WHEEL_GEAR_TEETH = 20; //# of teeth on the wheel gear
    public final double GEAR_RATIO = (MOTOR_GEAR_TEETH + 0.0) / WHEEL_GEAR_TEETH; //For every full turn of the motor, the wheel turns this many rotations.
    public final double MM_TO_INCHES =  25.4;
    public final double MOTOR_TO_INCHES = GEAR_RATIO * WHEEL_DIAMETER * Math.PI / MM_TO_INCHES; //For every full turn of both motors, the wheel moves forward this many inches
    public final double NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION = 537.6;
    public double intakeHalf = 0;
    public double drivetrainHalf = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        tankDrive.init(hardwareMap);

        waitForStart();


        while(opModeIsActive())
        {

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



            if(gamepad2.dpad_up)
                claw(false);
            else if (gamepad2.dpad_down)
                claw(true);


            if (gamepad1.x)
            {
                pidLinearMovement(100,0.001);
            }
            else if (gamepad1.y)
            {
                pidLinearMovement(100,0.0003);
            }
            else if (gamepad1.a)
            {
                pidLinearMovement(100,0.0001);
            }
            else if (gamepad1.b)
            {
                pidLinearMovement(100,0.00003);
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
            //d = ((error - errorPrev) / (time - timePrev)) /targetTick * kD;
            d = ((error - errorPrev) / (time - timePrev)) * kD;

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


    public double averageEncoderTick()
    {
        double output = tankDrive.LM0.getCurrentPosition() + tankDrive.LM1.getCurrentPosition() + tankDrive.RM0.getCurrentPosition() +tankDrive.RM0.getCurrentPosition();
        output /= 4.0;
        return output;
    }

    public double joystickAngle() {
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

    public void runMotor(double leftInput, double rightInput) {
        tankDrive.LM0.setPower(leftInput);
        tankDrive.LM1.setPower(leftInput);
        tankDrive.RM0.setPower(rightInput);
        tankDrive.RM1.setPower(rightInput);
    }
    public void freeze() {
        tankDrive.LM0.setPower(0);
        tankDrive.LM1.setPower(0);
        tankDrive.RM0.setPower(0);
        tankDrive.RM1.setPower(0);
    }
    public void runIntake(double power) {
        tankDrive.Intake1.setPower(power);
        tankDrive.Intake2.setPower(power);
    }
    public void intake()
    {
        if (gamepad2.a && intakeClock.seconds() > 0.2 & intakeHalf == 0.5)
        {
            intakeHalf = 1;
            intakeClock.reset();
            telemetry.addData("Current Intake Speed", intakeHalf);
            telemetry.update();
        }
        else if (gamepad2.a && intakeClock.seconds() > 0.5 & intakeHalf == 1)
        {
            intakeHalf = 0.5;
            intakeClock.reset();
            telemetry.addData("Current Intake Speed", intakeHalf);
            telemetry.update();
        }

        if (Math.abs(gamepad2.left_trigger) > 0.2)
            runIntake(gamepad2.left_trigger * intakeHalf);
        else if (Math.abs(gamepad2.right_trigger) > 0.2)
            runIntake(-gamepad2.right_trigger * intakeHalf);
        else
            runIntake(0);
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
