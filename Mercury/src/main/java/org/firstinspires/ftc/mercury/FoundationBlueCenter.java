package org.firstinspires.ftc.mercury;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings("ALL")
@Autonomous(name="Foundation Blue Center", group="blue")
//@Disabled
public class FoundationBlueCenter extends LinearOpMode {

    TankDriveALPHA tankDrive = new TankDriveALPHA();

    BNO055IMU imu;
    double globalAngle;
    PIDController linearPID = new PIDController(0, 0, 0);
    PIDController turningPID = new PIDController(0, 0, 0);
    Orientation lastAngles = new Orientation();

    ElapsedTime clock = new ElapsedTime();

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

        ElapsedTime et = new ElapsedTime();

        tankDrive.init(hardwareMap);

        tankDrive.RightNugget.setPosition(tankDrive.ROUT);
        tankDrive.LeftNugget.setPosition(tankDrive.LOUT);
        //tankDrive.LeftGate.setPosition(TankDriveALPHA.LEFT_GATE_UP_POS);
        tankDrive.RightGate.setPosition(TankDriveALPHA.RIGHT_GATE_UP_POS);
        tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_UP_POS);
        waitForStart();

        while(isStarted())
        {
            tankDrive.RightNugget.setPosition(tankDrive.RIN); // Movement 1
            tankDrive.LeftNugget.setPosition(tankDrive.LIN);
            tankDrive.fang(true);
            telemetry.addData("Left Range:", tankDrive.LeftRange.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Range:", tankDrive.RightRange.getDistance(DistanceUnit.INCH));
            telemetry.update();
            turnOneWheelDirection(90, 0.8, 0.4, 0.008,3, 'r');
            turnOneWheelDirection(-90, 0.5, 0.33, 0.008,3, 'l');

            telemetry.addData("Left Range:", tankDrive.LeftRange.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Range:", tankDrive.RightRange.getDistance(DistanceUnit.INCH));
            telemetry.update();
            //overshootLinearMovement(20,3);

            tankDrive.runMotor(0.3);
            et.reset();
            while (tankDrive.LeftRange.getDistance(DistanceUnit.INCH) > 5 && tankDrive.RightRange.getDistance(DistanceUnit.INCH) > 5 &&
                    et.seconds() < 5) {

            }

            freeze();
            telemetry.addData("Left Range:", tankDrive.LeftRange.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Range:", tankDrive.RightRange.getDistance(DistanceUnit.INCH));
            telemetry.update();
            sleep(250);
            tankDrive.fang(false);
            sleep(500);
            freeze();

            turnOneWheelDirection(90, 1.0, 1, 0.0111, 6, 'l'); // Movement 2
            tankDrive.fang(true);
            freeze();

            pidLinearMovement(25, 3);
            turnOneWheelDirection(45, 0.7, 0.4, 0.002, 4, 'l');
            pidLinearMovement(-15, 3);
            turnOneWheelDirection(-45, 0.7, 0.4, 0.002, 4, 'r');

            pidLinearMovement(-10, 4);
            ////////tankDrive.LeftGate.setPosition(TankDriveALPHA.LEFT_GATE_DOWN_POS);
            tankDrive.RightGate.setPosition(TankDriveALPHA.RIGHT_GATE_DOWN_POS);
            freeze();
            tankDrive.resetEncoders();
            sleep(500);
            while (opModeIsActive()) {
                tankDrive.stayInPlace();
            }
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


    public void pidLinearMovement(double distance, double timeframe)
    {
        double conversionIndex = 537.6/((26.0/20.0)*90.0* Math.PI / 25.4); // Ticks per inch
        double timeFrame = timeframe; //distance * distanceTimeIndex;
        double errorMargin = 5;
        double powerFloor = 0.25;
        double powerCeiling = 0.8;

        clock.reset();
        tankDrive.resetEncoders();

        double targetTick = distance / MOTOR_TO_INCHES * NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION *50/47;
        telemetry.addData("ticks", targetTick);
        telemetry.update();
        double error = targetTick;
        double errorPrev = 0;
        double kP = 1.5;
        double kD = 0.01;
        double p, d;
        double output;
        double time = clock.seconds();
        double timePrev = 0;


        while (clock.seconds() < timeFrame && Math.abs(error) > errorMargin && opModeIsActive())
        {
            //output = linearPID.PIDOutput(targetTick,averageEncoderTick(),clock.seconds());

            p = Math.abs(error/targetTick * kP);
            d = 0; //((error - errorPrev) / (time - timePrev)) / 1000 /targetTick * kD;

            output = p + d;
            output = Math.max(output, powerFloor);
            output = Math.min(output, powerCeiling);
            if (error < 0) output *= -1;
            runMotor(output, output);

            errorPrev = error;

            double tempAvg = targetTick > 0 ? tankDrive.getEncoderAvg(telemetry) : -tankDrive.getEncoderAvg(telemetry);
            error = targetTick - tempAvg;

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

    public void overshootLinearMovement(double distance, double timeframe)
    {
        double conversionIndex = 537.6/((26.0/20.0)*90.0* Math.PI / 25.4); // Ticks per inch
        double timeFrame = timeframe; //distance * distanceTimeIndex;
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
        double kD = 0.01;
        double p, d;
        double output;
        double time = clock.seconds();
        double timePrev = 0;


        while (clock.seconds() < timeFrame && error > 0 && opModeIsActive())
        {
            //output = linearPID.PIDOutput(targetTick,averageEncoderTick(),clock.seconds());

            p = Math.abs(error)/targetTick * kP;
            d = ((error - errorPrev) / (time - timePrev)) /targetTick * kD;

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
            telemetry.addData("RM0", tankDrive.RM0.getCurrentPosition());
            telemetry.addData("RM1", tankDrive.RM1.getCurrentPosition());
            telemetry.addData("LM0", tankDrive.LM0.getCurrentPosition());
            telemetry.addData("LM1", tankDrive.LM1.getCurrentPosition());
            telemetry.addData("error", error);
            telemetry.addData("kP", kP);
            telemetry.addData("output", output);
            telemetry.update();


        }
        tankDrive.runMotor(0,0);
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

            if (error < 0)
                leftPower *= -1;

            tankDrive.RM0.setPower(leftPower);
            tankDrive.RM1.setPower(leftPower);
            //tankDrive.RM0.setPower(rightPower);
            //tankDrive.RM1.setPower(rightPower);
        }

        // turn the motors off.
        tankDrive.LM0.setPower(0);
        tankDrive.LM1.setPower(0);
        tankDrive.RM0.setPower(0);
        tankDrive.RM1.setPower(0);

        // wait for rotation to stop.
        sleep(1000);
    }

    private void turnOneWheelDirection(int initial, double powerCeiling, double powerFloor, double kp, double t, char l)
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

            if (error < 0)
                leftPower *= -1;

            if (l == 'l') {
                tankDrive.LM0.setPower(-leftPower);
                tankDrive.LM1.setPower(-leftPower);
            }
            else {
                tankDrive.RM0.setPower(leftPower);
                tankDrive.RM1.setPower(leftPower);
            }
            //tankDrive.RM0.setPower(rightPower);
            //tankDrive.RM1.setPower(rightPower);
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
}
