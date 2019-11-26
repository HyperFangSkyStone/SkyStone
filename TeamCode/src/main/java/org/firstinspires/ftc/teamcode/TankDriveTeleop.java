package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="Tank", group="1")
//@Disabled
public class TankDriveTeleop extends LinearOpMode {

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

    int lIntakeServoPosition;
    int rIntakeServoPosition;

    @Override
    public void runOpMode() throws InterruptedException
    {
        tankDrive.init(hardwareMap);
        lIntakeServoPosition = 0;
        rIntakeServoPosition = 0;
        //tankDrive.PServo1.;
        //tankDrive.PServo2.setPosition(1);

        waitForStart();
        int pinger = 0;

        while(opModeIsActive()) {
            telemetry.addData("Pinger", ++pinger);
            //telemetry.update();
            /*if (gamepad1.left_bumper)
            {
                double forwardPower = -gamepad1.left_stick_y;
                double turnPower = gamepad1.left_stick_x;
                tankDrive.LM0.setPower(forwardPower + turnPower);
                tankDrive.LM1.setPower(forwardPower + turnPower);
                tankDrive.RM0.setPower(forwardPower - turnPower);
                tankDrive.RM1.setPower(forwardPower - turnPower);
            }*/
            if (Math.abs(gamepad1.left_stick_y) > 0.2) {
                tankDrive.LM0.setPower(-gamepad1.left_stick_y);
                tankDrive.LM1.setPower(-gamepad1.left_stick_y);
            } else {
                tankDrive.LM0.setPower(0);
                tankDrive.LM1.setPower(0);
            }

            if (Math.abs(gamepad1.right_stick_y) > 0.2) {
                tankDrive.RM0.setPower(-gamepad1.right_stick_y);
                tankDrive.RM1.setPower(-gamepad1.right_stick_y);
            } else {
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
            telemetry.addData("Intake", ++pinger);
            //telemetry.update();

            if (Math.abs(gamepad2.left_stick_y) > 0.2) {
                if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                    tankDrive.Intake2.setPower(-gamepad2.left_stick_y / 2);
                } else {
                    tankDrive.Intake2.setPower(-gamepad2.left_stick_y);
                }
            } else {
                tankDrive.Intake2.setPower(0);
            }

            if (Math.abs(gamepad2.right_stick_y) > 0.2) {
                if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                    tankDrive.Intake1.setPower(-gamepad2.right_stick_y / 2);
                } else {
                    tankDrive.Intake1.setPower(-gamepad2.right_stick_y);
                }
            } else {
                tankDrive.Intake1.setPower(0);
            }


            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                telemetry.addData("Intake Operating at", " Half Speed");
            } else {
                telemetry.addData("Intake Operating at", " Full Speed");
            }
            //telemetry.update();

            telemetry.addData("Claw", ++pinger);
            //telemetry.update();
            if (gamepad2.left_bumper) {
                claw(false);
            } else if (gamepad2.right_bumper) {
                claw(true);
            }



            telemetry.addData("pidLinearMovement", ++pinger);
            //telemetry.update();
            if (gamepad1.x)
                pidLinearMovement(80, 0.1);
            else if (gamepad1.y)
                pidLinearMovement(20, 0.1);


            telemetry.addData("IntakeServos", ++pinger);
            //telemetry.update();
            intakeServos(); // Why do we have two things for intake?


            //for lift

            telemetry.addData("Lift", ++pinger);
            //telemetry.update();
            if (gamepad2.right_trigger > 0.1) {
                tankDrive.Lift1.setPower(gamepad2.right_trigger);
                tankDrive.Lift2.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0.1) {
                tankDrive.Lift1.setPower(-gamepad2.left_trigger);
                tankDrive.Lift2.setPower(-gamepad2.left_trigger);
            } else {
                tankDrive.Lift1.setPower(0);
                tankDrive.Lift2.setPower(0);
            }

            if (gamepad2.y) {
                int targ = 250; //Target
                double kp = 0.003;
                double powerFloor = 0.14;
                int errMarg = 10;
                while (Math.abs(tankDrive.Lift1.getCurrentPosition() - targ) < errMarg && Math.abs(tankDrive.Lift1.getCurrentPosition() - targ) < errMarg) {
                    int error = targ - Math.max(tankDrive.Lift1.getCurrentPosition(), tankDrive.Lift2.getCurrentPosition());
                    double porg = Math.abs(error) * kp;
                    porg = Math.max(porg, powerFloor);
                    if (error < 0)
                        porg *= -1;
                    tankDrive.Lift1.setPower(porg);
                    tankDrive.Lift2.setPower(porg);
                }
            }
            telemetry.addData("Lift 1:", tankDrive.Lift1.getCurrentPosition());
            telemetry.addData("Lift 2:", tankDrive.Lift2.getCurrentPosition());
            //telemetry.update();

            //for the the extender servos on lifts
            /*if (gamepad2.a && (tankDrive.PServo1.getPosition() != 1) && (tankDrive.PServo2.getPosition() != 0)) {
                tankDrive.PServo1.setPosition(tankDrive.PServo1.getPosition() + .1);
                tankDrive.PServo2.setPosition(tankDrive.PServo2.getPosition() - .1);
                telemetry.addData("IN PServo 1 position: ", tankDrive.PServo1.getPosition() );
                telemetry.addData("IN PServo 2 position: ", tankDrive.PServo2.getPosition() );
                telemetry.update();
            }
            else if (gamepad2.b && (tankDrive.PServo1.getPosition() != 0) && (tankDrive.PServo2.getPosition() != 1)) {
                tankDrive.PServo1.setPosition(tankDrive.PServo1.getPosition() - .1);
                tankDrive.PServo2.setPosition(tankDrive.PServo2.getPosition() + .1);
                telemetry.addData("IN PServo 1 position: ", tankDrive.PServo1.getPosition() );
                telemetry.addData("IN PServo 2 position: ", tankDrive.PServo2.getPosition() );
                telemetry.update();
            }
            */
            /*else
            {
                tankDrive.PServo1.setPosition(tankDrive.PServo1.getPosition() - 0);
                tankDrive.PServo2.setPosition(tankDrive.PServo2.getPosition() + 0);
                telemetry.addData("STAWP", tankDrive.PServo1.getPosition() );
                telemetry.addData("STAWP", tankDrive.PServo2.getPosition() );
                telemetry.update();

            }*/


            telemetry.addData("PServo", ++pinger);
            //telemetry.update();
                if (gamepad2.right_stick_button ) {
                    tankDrive.PServo1.setPower(.5);
                    telemetry.addData("PServo 1 UP :: ", tankDrive.PServo1.getPower());

                }
                else if (gamepad2.left_stick_button) {
                    tankDrive.PServo1.setPower(-.5);
                    telemetry.addData("PServo 1 DOWN :: ", tankDrive.PServo1.getPower());
                } else {
                    tankDrive.PServo1.setPower(0);
                    telemetry.addData("PServo 1 STAWP!!! :: ", tankDrive.PServo1.getPower());
                }
            telemetry.update();




            //if (gamepad2.x){
              //  telemetry.addData("IN PServo 1 position: ", tankDrive.PServo1.getPosition() );
                //telemetry.addData("IN PServo 2 position: ", tankDrive.PServo2.getPosition() );
                //telemetry.update();
            //}

        }

        //freeze();

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
        double kP = 0.8;
        double kD = kd;
        double p, d;
        double output;
        double time = clock.seconds();
        double timePrev = 0;


        while (clock.seconds() < timeFrame && Math.abs(error) > errorMargin && opModeIsActive())
        {
            //output = linearPID.PIDOutput(targetTick,averageEncoderTick(),clock.seconds());

            p = Math.abs(error)/targetTick * kP;
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

    public void claw(boolean x) {
        if (x) {
            tankDrive.LServo.setPosition(1); //true = up
            tankDrive.RServo.setPosition(0);
        } else {
            tankDrive.LServo.setPosition(0); //false = down
            tankDrive.RServo.setPosition(1);
        }
    }

    private void intakeServos() {
        if (gamepad2.dpad_down && lIntakeServoPosition != 0 && rIntakeServoPosition != 0)
        {
            lIntakeServoPosition = 0;
            rIntakeServoPosition = 0;
            tankDrive.RServo2.setPosition(0.0);
            tankDrive.LServo2.setPosition(1.0);
        }

        else if (gamepad2.dpad_up && lIntakeServoPosition != 1 && rIntakeServoPosition != 1) {
            lIntakeServoPosition = 1;
            rIntakeServoPosition = 1;
            tankDrive.RServo2.setPosition(1.0);
            tankDrive.LServo2.setPosition(0.0);
        }

        else if (gamepad2.dpad_right && lIntakeServoPosition != 0 && rIntakeServoPosition != 1)
        {
            lIntakeServoPosition = 0;
            rIntakeServoPosition = 1;
            tankDrive.RServo2.setPosition(1.0);
            tankDrive.LServo2.setPosition(1.0);
        }

        else if (gamepad2.dpad_left && lIntakeServoPosition != 1 && rIntakeServoPosition != 0)
        {
            lIntakeServoPosition = 1;
            rIntakeServoPosition = 0;
            tankDrive.RServo2.setPosition(0.0);
            tankDrive.LServo2.setPosition(0.0);
        }
    }
}
