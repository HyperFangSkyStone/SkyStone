package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings("ALL")
@Autonomous(name="Foundation Blue", group="blue")
//@Disabled
public class TankDriveAutoFoundationBlue extends LinearOpMode {

    TankDriveALPHA tankDrive = new TankDriveALPHA();

    BNO055IMU imu;
    double globalAngle;
    PIDController linearPID = new PIDController(0, 0, 0);
    PIDController turningPID = new PIDController(0, 0, 0);
    Orientation lastAngles = new Orientation();

    public static int skystonePosition = 0;
    public static double distanceTimeIndex = 0; //time expected per inch
    public static double angleTimeIndex = 0; //time expected per degree

    public final double WHEEL_DIAMETER = 90; //Wheel diameter in mm
    public final int MOTOR_GEAR_TEETH = 26; //# of teeth on the motor gear
    public final int WHEEL_GEAR_TEETH = 20; //# of teeth on the wheel gear
    public final double GEAR_RATIO = (MOTOR_GEAR_TEETH + 0.0) / WHEEL_GEAR_TEETH; //For every full turn of the motor, the wheel turns this many rotations.
    public final double MM_TO_INCHES =  25.4;
    public final double MOTOR_TO_INCHES = GEAR_RATIO * WHEEL_DIAMETER * Math.PI / MM_TO_INCHES; //For every full turn of both motors, the wheel moves forward this many inches
    public final double NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION = 537.6;

    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        tankDrive.init(hardwareMap);
        waitForStart();

        while(isStarted())
        {
            runIntake(1);
            sleep(750);
            runIntake(0);
            claw(true);
            moveOvershoot(35, 2.5, 0.001, 0.3);
            freeze();
            sleep(500);
            claw(false);
            sleep(2000);
            freeze();
            //movethForward(-15, 2.5, 0.005, 0.4);
            turnOneWheelDirection(90, 0.8, 0.6, 0.005, 4);
            //turnOneWheelDirection(-45, 0.8, 0.6, 0.005, 4);
            //turnethDirection(-45, 0.6, 0.3, 0.005, 4);
            claw(true);
            freeze();
            sleep(1500);
            movethForward(30, 3, 0.005, 0.4);
            freeze();
            sleep(1000);
            movethForward(-30, 2.5, 0.005, 0.4);
            freeze();
            break;
        }
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

    public double sigmoid(double error, double ceiling, double floor, double half, double stiff) {
        return floor + (ceiling - floor) / (1 + Math.pow(Math.E, stiff * (half - error)));
    }

    public void movethForward(double inches, double t, double kp, double power)
    {
        tankDrive.resetEncoders();
        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        double encoderTicks = inches / MOTOR_TO_INCHES * NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION; // target number of encoder tick
        int errorMargin = 5; //Amount of error in ticks we are willing to accept
        double powerFloor = 0.19; //Minimum power
        double powerCeiling = power; //Maximum power

        telemetry.addData("ticks", encoderTicks);
        telemetry.update();
        while (Math.abs(tankDrive.getAverageEncoder('l') + tankDrive.getAverageEncoder('r') - encoderTicks * 2) > errorMargin && clock.seconds() < t && opModeIsActive())
        {
            if (Math.abs(encoderTicks - tankDrive.getAverageEncoder('l')) >= errorMargin)
            {
                //double leftpower = Math.abs(encoderTicks - tank.getAverageEncoder('l')) * kp;
                double leftpower = sigmoid(encoderTicks - tankDrive.getAverageEncoder('l'), powerCeiling, powerFloor, 800, kp * 4);
                leftpower = Math.min(leftpower, powerFloor);
                leftpower = Math.max(leftpower, powerCeiling);
                if (encoderTicks - tankDrive.getAverageEncoder('l') < 0) leftpower *= -1;
                tankDrive.LM0.setPower(leftpower);
                tankDrive.LM1.setPower(leftpower);
                telemetry.addData("LeftPower", leftpower);
            }
            else {
                tankDrive.LM0.setPower(0);
                tankDrive.LM1.setPower(0);
            }

            if (Math.abs(encoderTicks - tankDrive.getAverageEncoder('r')) >= errorMargin)
            {
                //double rightpower = Math.abs(encoderTicks - tank.getAverageEncoder('r')) * kp;
                double rightpower = sigmoid(encoderTicks - tankDrive.getAverageEncoder('r'), powerCeiling, powerFloor, 800, kp * 6);
                rightpower = Math.min(rightpower, powerFloor);
                rightpower = Math.max(rightpower, powerCeiling);
                if (encoderTicks - tankDrive.getAverageEncoder('r') < 0) rightpower *= -1;
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
            telemetry.addData("ticks", encoderTicks);
            telemetry.addData("Current L", tankDrive.getAverageEncoder('l'));
            telemetry.addData("Current R", tankDrive.getAverageEncoder('l'));
            telemetry.addData("kP", kp);
            telemetry.update();

        }
    }


    public void moveOvershoot(double inches, double t, double kp, double power)
    {
        tankDrive.resetEncoders();
        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        double encoderTicks = inches / MOTOR_TO_INCHES * NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION; // target number of encoder tick
        int errorMargin = 5; //Amount of error in ticks we are willing to accept
        double powerFloor = 0.19; //Minimum power
        double powerCeiling = power; //Maximum power

        telemetry.addData("ticks", encoderTicks);
        telemetry.update();
        while (Math.abs(tankDrive.getAverageEncoder('l')) + Math.abs(tankDrive.getAverageEncoder('r')) < (encoderTicks - errorMargin) * 2 && clock.seconds() < t && opModeIsActive())
        {
            if (Math.abs(encoderTicks - tankDrive.getAverageEncoder('l')) > errorMargin)
            {
                //double leftpower = Math.abs(encoderTicks - tank.getAverageEncoder('l')) * kp;
                double leftpower = sigmoid(encoderTicks - tankDrive.getAverageEncoder('l'), powerCeiling, powerFloor, 800, kp * 4);
                leftpower = Math.min(leftpower, powerFloor);
                leftpower = Math.max(leftpower, powerCeiling);
                if (encoderTicks - tankDrive.getAverageEncoder('l') < 0) leftpower *= -1;
                tankDrive.LM0.setPower(leftpower);
                tankDrive.LM1.setPower(leftpower);
                telemetry.addData("LeftPower", leftpower);
            }
            else {
                tankDrive.LM0.setPower(0);
                tankDrive.LM1.setPower(0);
            }

            if (Math.abs(encoderTicks - tankDrive.getAverageEncoder('r')) > errorMargin)
            {
                //double rightpower = Math.abs(encoderTicks - tank.getAverageEncoder('r')) * kp;
                double rightpower = sigmoid(encoderTicks - tankDrive.getAverageEncoder('r'), powerCeiling, powerFloor, 800, kp * 6);
                rightpower = Math.min(rightpower, powerFloor);
                rightpower = Math.max(rightpower, powerCeiling);
                if (encoderTicks - tankDrive.getAverageEncoder('r') < 0) rightpower *= -1;
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
            telemetry.addData("ticks", encoderTicks);
            telemetry.addData("Current L", tankDrive.getAverageEncoder('l'));
            telemetry.addData("Current R", tankDrive.getAverageEncoder('l'));
            telemetry.addData("kP", kp);
            telemetry.update();
        }
    }

    private void turnethDirection(int initial, double powerCeiling, double powerFloor, double kp, double t)
    {
        lastAngles = imu.getAngularOrientation();
        double currentAngle = lastAngles.firstAngle;
        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        globalAngle += initial;
        if(globalAngle > 180)
            globalAngle -=360;
        if (globalAngle < -180)
            globalAngle += 360;
        double  leftPower, rightPower;

        // restart imu movement tracking.
        //resetAngle();

        telemetry.addData("currentAngle", currentAngle);
        telemetry.update();
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // set power to rotate.

        // rotate until turn is completed.
        while (clock.seconds() < t && Math.abs(globalAngle - imu.getAngularOrientation().firstAngle) > 3 && opModeIsActive())
        {
            lastAngles = imu.getAngularOrientation();
            currentAngle = lastAngles.firstAngle;

            telemetry.addData("CurrentAngle", currentAngle);
            telemetry.update();

            double error = globalAngle - currentAngle;
            if (error > 180)
                error -= 360;
            if (error < -180)
                error += 360;
            double rawPower = Math.abs(error) * kp;
            rawPower = Math.max(rawPower, powerFloor);
            rawPower = Math.min(rawPower, powerCeiling);
            leftPower = rawPower;
            rightPower = rawPower;
            if (error > 0)
                leftPower *= -1;
            else
                rightPower *= -1;

            tankDrive.LM0.setPower(leftPower);
            tankDrive.LM1.setPower(leftPower);
            tankDrive.RM0.setPower(rightPower);
            tankDrive.RM1.setPower(rightPower);
        }

        // turn the motors off.
        tankDrive.LM0.setPower(0);
        tankDrive.LM1.setPower(0);
        tankDrive.RM0.setPower(0);
        tankDrive.RM1.setPower(0);

        // wait for rotation to stop.
        sleep(1000);
    }

    private void turnOneWheelDirection(int initial, double powerCeiling, double powerFloor, double kp, double t)
    {
        lastAngles = imu.getAngularOrientation();
        double currentAngle = lastAngles.firstAngle;
        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        globalAngle += initial;
        if(globalAngle > 180)
            globalAngle -=360;
        if (globalAngle < -180)
            globalAngle += 360;
        double  leftPower, rightPower;

        // restart imu movement tracking.
        //resetAngle();

        telemetry.addData("currentAngle", currentAngle);
        telemetry.update();
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // set power to rotate.

        // rotate until turn is completed.
        while (clock.seconds() < t && Math.abs(globalAngle - imu.getAngularOrientation().firstAngle) > 3 && opModeIsActive())
        {
            lastAngles = imu.getAngularOrientation();
            currentAngle = lastAngles.firstAngle;

            telemetry.addData("CurrentAngle", currentAngle);
            telemetry.update();

            double error = globalAngle - currentAngle;
            if (error > 180)
                error -= 360;
            if (error < -180)
                error += 360;
            double rawPower = Math.abs(error) * kp;
            rawPower = Math.max(rawPower, powerFloor);
            rawPower = Math.min(rawPower, powerCeiling);
            leftPower = rawPower;
            rightPower = rawPower;

            if (error < 0) {
                leftPower *= -1;
                rightPower *= -1;
            }

            //tankDrive.RM0.setPower(leftPower);
            //tankDrive.RM1.setPower(leftPower);
            tankDrive.LM0.setPower(-leftPower);
            tankDrive.LM1.setPower(-leftPower);
        }

        // turn the motors off.
        tankDrive.LM0.setPower(0);
        tankDrive.LM1.setPower(0);
        tankDrive.RM0.setPower(0);
        tankDrive.RM1.setPower(0);

        // wait for rotation to stop.
        sleep(1000);
    }

    public double averageEncoderTick()
    {
        double output = tankDrive.LM0.getCurrentPosition() + tankDrive.LM1.getCurrentPosition() + tankDrive.RM0.getCurrentPosition() +tankDrive.RM0.getCurrentPosition();
        output /= 4.0;
        return output;
    }

    public void runIntake(double input)
    {
        tankDrive.Intake1.setPower(input);
        tankDrive.Intake2.setPower(input);
    }

    public void claw(boolean x)
    {
        if (x) {
            tankDrive.LServo.setPosition(1); //true = up
            tankDrive.RServo.setPosition(0);
        }
        else {
            tankDrive.LServo.setPosition(0); //false = down
            tankDrive.RServo.setPosition(1);
        }
    }
}
