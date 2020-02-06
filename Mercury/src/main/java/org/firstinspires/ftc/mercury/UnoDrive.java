package org.firstinspires.ftc.mercury;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="UnoDrive", group="1")
@Disabled
public class UnoDrive extends LinearOpMode {

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
    public final double MM_TO_INCHES = 25.4;
    public final double MOTOR_TO_INCHES = GEAR_RATIO * WHEEL_DIAMETER * Math.PI / MM_TO_INCHES; //For every full turn of both motors, the wheel moves forward this many inches
    public final double NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION = 537.6;

    public final double LIFT_HEIGHT_TO_PICK_UP_BLOCKS_INCHES = .5;


    int lIntakeServoPosition;
    int rIntakeServoPosition;

    DcMotor LM0 = tankDrive.LM0;
    DcMotor LM1 = tankDrive.LM1;
    DcMotor RM0 = tankDrive.RM0;
    DcMotor RM1 = tankDrive.RM1;

    final double LIFT_ENCODER_TICKS_PER_INCH = 82;

    int towerPosition = 0;
    boolean dpadright2Ispressed = false;
    boolean dpadleft2Ispressed = false;

    double conversionFunction = Math.pow(10, -30);

    ElapsedTime et = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        tankDrive.init(hardwareMap);
        lIntakeServoPosition = 0;
        rIntakeServoPosition = 0;
        tankDrive.RightNugget.setPosition(tankDrive.RIN);
        tankDrive.LeftNugget.setPosition(tankDrive.LIN);

        DcMotor LM0 = tankDrive.LM0;
        DcMotor LM1 = tankDrive.LM1;
        DcMotor RM0 = tankDrive.RM0;
        DcMotor RM1 = tankDrive.RM1;


        waitForStart();

        boolean leftpressed = false;
        boolean rightpressed = false;


        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                tankDrive.LM0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                tankDrive.LM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                tankDrive.RM0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                tankDrive.RM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else if (gamepad1.dpad_down) {
                tankDrive.LM0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                tankDrive.LM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                tankDrive.RM0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                tankDrive.RM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else  {
                if (gamepad1.dpad_left && !leftpressed) {
                    if (tankDrive.LM0.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE) {
                        tankDrive.LM0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        tankDrive.LM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    } else {
                        tankDrive.LM0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        tankDrive.LM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                    leftpressed = true;
                } else {
                    leftpressed = false;
                }

                if (gamepad1.dpad_right && !rightpressed) {
                    if (tankDrive.RM0.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE) {
                        tankDrive.RM0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        tankDrive.RM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    } else {
                        tankDrive.RM0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        tankDrive.RM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                    rightpressed = true;
                } else {
                    rightpressed = false;
                }
            }


            if (Math.abs(gamepad1.left_stick_y) > 0.2) {
                LM0.setPower(-gamepad1.left_stick_y);
                LM1.setPower(-gamepad1.left_stick_y);
            } else {
                LM0.setPower(0);
                LM1.setPower(0);
            }

            if (Math.abs(gamepad1.right_stick_y) > 0.2) {
                RM0.setPower(-gamepad1.right_stick_y);
                RM1.setPower(-gamepad1.right_stick_y);
            } else {
                RM0.setPower(0);
                RM1.setPower(0);
            }


        }

        tankDrive.freeze();


    }


    //  ++++++ Helper Methods ++++++

    public void linearMovement(double inputPower, double t) {

        clock.reset();

        while (clock.seconds() < t) {
            LM0.setPower(inputPower);
            LM1.setPower(inputPower);
            RM0.setPower(inputPower);
            RM1.setPower(inputPower);
            telemetry.update();
        }
        LM0.setPower(0);
        LM1.setPower(0);
        RM0.setPower(0);
        RM1.setPower(0);
    }

    public void pidLinearMovement(double distance, double kd) {
        double conversionIndex = 537.6 / ((26.0 / 20.0) * 90.0 * Math.PI / 25.4); // Ticks per inch
        double timeFrame = 5; //distance * distanceTimeIndex;
        double errorMargin = 5;
        double powerFloor = 0.2;
        double powerCeiling = 0.8;

        clock.reset();
        tankDrive.resetEncoders();

        double targetTick = distance / MOTOR_TO_INCHES * NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION * 50 / 47;
        telemetry.addData("ticks", targetTick);
        telemetry.update();
        double error = targetTick;
        double errorPrev = 0;
        double kP = 0.8;
        double kD = kd;
        double p, d;
        double output;
        double time = clock.seconds();
        double timePrev = 0;


        while (clock.seconds() < timeFrame && Math.abs(error) > errorMargin && opModeIsActive()) {
            //output = linearPID.PIDOutput(targetTick,averageEncoderTick(),clock.seconds());

            p = Math.abs(error) / targetTick * kP;
            d = ((error - errorPrev) / (time - timePrev)) / targetTick * kD;

            output = p + d;
            output = Math.max(output, powerFloor);
            output = Math.min(output, powerCeiling);
            if (error < 0) output *= -1;
            tankDrive.runMotor(output, output);

            errorPrev = error;
            error = targetTick - tankDrive.getEncoderAvg(telemetry);

            timePrev = time;
            time = clock.seconds();

            telemetry.addData("Target", targetTick);
            telemetry.addData("Current", averageEncoderTick());
            telemetry.addData("RM0", RM0.getCurrentPosition());
            telemetry.addData("RM1", RM1.getCurrentPosition());
            telemetry.addData("LM0", LM0.getCurrentPosition());
            telemetry.addData("LM1", LM1.getCurrentPosition());
            telemetry.addData("error", error);
            telemetry.addData("kP", kP);
            telemetry.addData("output", output);
            telemetry.update();


        }
        tankDrive.runMotor(0, 0);
    }


    public void turnToAngle(double angle) {

    }

    public double averageEncoderTick() {
        double output = tankDrive.LM0.getCurrentPosition() + tankDrive.LM1.getCurrentPosition() + tankDrive.RM0.getCurrentPosition() + tankDrive.RM0.getCurrentPosition();
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
            angle = Math.atan(x / y) * 180 / Math.PI;
        else if (x < 0 && y > 0)
            angle = Math.atan(x / y) * 180 / Math.PI;
        else if (x < 0 && y < 0)
            angle = Math.atan(x / y) * 180 / Math.PI - 180;
        else
            angle = Math.atan(x / y) * 180 / Math.PI + 180;
        return -angle;
    }


    private void intake() //intake wheels & nuggets manipulation
    {
        if (Math.abs(gamepad2.left_stick_y) > 0.2)
            tankDrive.Intake2.setPower(-gamepad2.left_stick_y);
        else
            tankDrive.Intake2.setPower(0);

        if (Math.abs(gamepad2.right_stick_y) > 0.2)
            tankDrive.Intake1.setPower(-gamepad2.right_stick_y);
        else
            tankDrive.Intake1.setPower(0);

        if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
            lIntakeServoPosition = 1;
            rIntakeServoPosition = 1;
            tankDrive.RightNugget.setPosition(tankDrive.ROUT);
            tankDrive.LeftNugget.setPosition(tankDrive.LOUT);
            telemetry.addData("Intake Nuggets", " Open");
        } else {
            lIntakeServoPosition = 0;
            rIntakeServoPosition = 0;
            tankDrive.RightNugget.setPosition(tankDrive.RIN);
            tankDrive.LeftNugget.setPosition(tankDrive.LIN);
            telemetry.addData("Intake Nuggets", " Closed");
        }

    }


    private void liftererPID(int position) {
        // Position 0 is foundation (2.25 inches), position 1, 2, 3, etc. are block heights of the EXISTING tower
        double inches = position * 4 + 4.7;
        double targ = inches * LIFT_ENCODER_TICKS_PER_INCH;

        double kp = 0.003;
        double kI = 0.000002;
        double kd = 4 * conversionFunction;

        double powerFloor = 0.3 + 0.02 * position;
        double powerCeiling = 0.8;

        double I = 0;
        double prevError = 0;
        double d = 0;

        int errMarg = 20;
        double timeOut = 3.0;
        et.reset();
        while (Math.abs(tankDrive.Lift1.getCurrentPosition() - targ) > errMarg && Math.abs(tankDrive.Lift1.getCurrentPosition() - targ) > errMarg && opModeIsActive() && et.seconds() < timeOut) {
            double error = targ - Math.max(Math.abs(tankDrive.Lift1.getCurrentPosition()), Math.abs(tankDrive.Lift2.getCurrentPosition()));
            I += error;
            d = (error - prevError) * 10;
            double porg = Math.abs(error) * kp + I * kI + kd * d;
            porg = Math.max(porg, powerFloor);
            porg = Math.min(porg, powerCeiling);
            if (error < 0)
                porg *= -1;
            tankDrive.Lift1.setPower(porg);
            tankDrive.Lift2.setPower(porg);

            telemetry.addData("Porg:", porg);
            telemetry.addData("Lift 1:", tankDrive.Lift1.getCurrentPosition());
            telemetry.addData("Lift 2:", tankDrive.Lift2.getCurrentPosition());
            telemetry.addData("I:", I);
            telemetry.addData("Error:", error);
            telemetry.update();
            prevError = error;
        }
        tankDrive.Lift1.setPower(0);
        tankDrive.Lift2.setPower(0);

    }

    private void setLiftToZero() {


        double targ = 0;
        double kp = 0.003;
        double kI = 2 * conversionFunction;
        double kd = 4 * conversionFunction;
        double powerFloor = 0.35;
        double I = 0;
        double prevError = 0;
        double d = 0;
        int errMarg = 10;

        et.reset();
        double timeout = 3;

        while (Math.abs(tankDrive.Lift1.getCurrentPosition() - targ) > errMarg && Math.abs(tankDrive.Lift1.getCurrentPosition() - targ) > errMarg && opModeIsActive() && et.seconds() < timeout) {
            double error = targ - Math.max(Math.abs(tankDrive.Lift1.getCurrentPosition()), Math.abs(tankDrive.Lift2.getCurrentPosition()));
            I += error;
            d = (error - prevError) * 10;
            double porg = Math.abs(error) * kp + I * kI + kd * d;
            porg = Math.max(porg, powerFloor);
            if (error < 0)
                porg *= -1;
            tankDrive.Lift1.setPower(porg);
            tankDrive.Lift2.setPower(porg);

            telemetry.addData("Porg:", porg);
            telemetry.addData("Lift 1:", tankDrive.Lift1.getCurrentPosition());
            telemetry.addData("Lift 2:", tankDrive.Lift2.getCurrentPosition());
            telemetry.addData("Error:", error);
            telemetry.update();
        }
        tankDrive.Lift1.setPower(0);
        tankDrive.Lift2.setPower(0);
    }

    public void liftToPickUpPosition() {

        double targ = 135;//LIFT_ENCODER_TICKS_PER_INCH * LIFT_HEIGHT_TO_PICK_UP_BLOCKS_INCHES;
        double error = targ - Math.max(Math.abs(tankDrive.Lift1.getCurrentPosition()), Math.abs(tankDrive.Lift2.getCurrentPosition()));
        while (Math.abs(error) > 10 && opModeIsActive()) {
            error = targ - Math.max(Math.abs(tankDrive.Lift1.getCurrentPosition()), Math.abs(tankDrive.Lift2.getCurrentPosition()));
            double kp = 0.015;
            double powerFloor = 0.25;
            double powerCeiling = 0.55;
            double pow = kp * Math.abs(error);
            pow = Math.max(pow, powerFloor);
            pow = Math.min(pow, powerCeiling);
            if (error < 0) {
                pow *= -1;
            }
            tankDrive.Lift1.setPower(pow);
            tankDrive.Lift2.setPower(pow);

            telemetry.addData("Pow:", pow);
            telemetry.addData("Lift 1:", tankDrive.Lift1.getCurrentPosition());
            telemetry.addData("Lift 2:", tankDrive.Lift2.getCurrentPosition());
            telemetry.addData("Error:", error);
            telemetry.update();
        }

        tankDrive.Lift1.setPower(0);
        tankDrive.Lift2.setPower(0);

    }


    public void resetLiftEncoder() {
        tankDrive.Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tankDrive.Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tankDrive.Lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tankDrive.Lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
