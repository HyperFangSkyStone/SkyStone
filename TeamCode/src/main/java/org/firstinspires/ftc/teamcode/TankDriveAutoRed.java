package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings("ALL")
@Autonomous(name="Skystone Red", group="red")
@Disabled
public class TankDriveAutoRed extends LinearOpMode {

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
    public final double RED_DIVIDER_ONE = 0;
    public final double RED_DIVIDER_TWO = 0;

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
        //waitForStart();

        VisionBitMapping vbm = new VisionBitMapping(this);

        while (!isStarted())
        {
            skystonePosition = 0;
            double avgX = vbm.avgX();
            if(avgX < RED_DIVIDER_ONE)
                skystonePosition = 1;
            else if (avgX < RED_DIVIDER_TWO)
                skystonePosition = 2;
            else
                skystonePosition = 3;

            telemetry.addData("Skystone Pos", skystonePosition);
            telemetry.update();
            if (skystonePosition == 0) {
                telemetry.addData("ERROR", "SkyStone Not Found.");
                telemetry.update();
            }
        }

        waitForStart();
        while(isStarted())
        {
            runIntake(1);
            sleep(750);
            runIntake(0);
            if (skystonePosition == 1) //Left
            {
                /*movethForward(24, 2.5, 0.001, 0.5);
                freeze();
                turnethDirection(10,0.35, 0.2, 0.005, 3);
                runIntake(-0.5);
                //tankDrive.getEncoderAvg(telemetry);
                //telemetry.update();
                //sleep(1000);
                movethForward(20, 2.5, 0.001, 0.4);
                sleep(250);
                runIntake(0);
                movethForward(-30, 2.5, 0.001, 0.4);
                runIntake(0.5);
                sleep(110);
                runIntake(0);
                freeze();
                turnethDirection(80,0.4, 0.2, 0.005, 3);
                movethForward(-5, 0.5,0.001, 0.8, 0.4);*/

                turnethDirection(10, 0.35, 0.2, 0.005, 3);
                freeze();

                tankDrive.RightNugget.setPosition(tankDrive.ROUT); //Collecting skystone
                tankDrive.LeftNugget.setPosition(tankDrive.LOUT);
                movethForward(34, 4, 0.001,0.5);
                tankDrive.Intake1.setPower(1.0);
                tankDrive.Intake2.setPower(1.0);
                tankDrive.RightNugget.setPosition(tankDrive.RIN);
                tankDrive.LeftNugget.setPosition(tankDrive.LIN);
                movethForward(5, 1.2, 0.001, 0.5);
                tankDrive.Intake1.setPower(0);
                tankDrive.Intake2.setPower(0);
                freeze();

                movethForward(-14, 1.5, 0.003, 0.5); //Delivering to building zone
                turnethDirection(80, 0.5, 0.2, 0.003, 3);
                movethForward(40, 3, 0.003, 0.5);
                tankDrive.Intake1.setPower(-1.0);
                tankDrive.Intake2.setPower(-1.0);
                sleep(3000);
                tankDrive.Intake1.setPower(0);
                tankDrive.Intake2.setPower(0);
                movethForward(-85, 5, 0.003, 0.7); //Slam into wall
                turnOneWheelDirection(-80, 0.5, 0.2, 0.003, 3);

                tankDrive.RightNugget.setPosition(tankDrive.ROUT); //Collecting skystone
                tankDrive.LeftNugget.setPosition(tankDrive.LOUT);
                movethForward(34, 2.5, 0.001,0.5);
                tankDrive.Intake1.setPower(1.0);
                tankDrive.Intake2.setPower(1.0);
                tankDrive.RightNugget.setPosition(tankDrive.RIN);
                tankDrive.LeftNugget.setPosition(tankDrive.LIN);
                movethForward(5, 1.2, 0.001, 0.5);
                tankDrive.Intake1.setPower(0);
                tankDrive.Intake2.setPower(0);
                freeze();

                movethForward(-14, 1.5, 0.003, 0.5); //Going back
                turnethDirection(80, 0.5, 0.2, 0.003, 3);
                movethForward(75, 5, 0.003, 0.5); // Delivering to building zone
                tankDrive.Intake1.setPower(-1.0);
                tankDrive.Intake2.setPower(-1.0);
                sleep(3000);
                tankDrive.Intake1.setPower(0);
                tankDrive.Intake2.setPower(0);
                movethForward(-5, 1, 0.001, 0.4);
            }
            else if (skystonePosition == 2)
            {
                /*movethForward(3, 0.5, 0.001, 0.3);
                turnethDirection(-90, 0.5, 0.3, 0.003, 3);
                turnOneWheelDirection(90, 0.5, 0.3, 0.003, 3);
                movethForward(10, 2, 0.001, 0.8, 0.25);
                freeze();
                turnethDirection(8, 0.5, 0.3, 0.003, 3);
                runIntake(-0.75);
                movethForward(26, 2, 0.001, 0.6, 0.25);
                sleep(500);
                runIntake(0);
                movethForward(-30, 2.5, 0.001, 0.4, 0.3);
                freeze();
                turnethDirection(82,0.35, 0.25, 0.005, 3);*/

                tankDrive.RightNugget.setPosition(tankDrive.ROUT); //Collecting skystone
                tankDrive.LeftNugget.setPosition(tankDrive.LOUT);
                movethForward(34, 4, 0.001,0.5);
                tankDrive.Intake1.setPower(1.0);
                tankDrive.Intake2.setPower(1.0);
                tankDrive.RightNugget.setPosition(tankDrive.RIN);
                tankDrive.LeftNugget.setPosition(tankDrive.LIN);
                movethForward(5, 1.2, 0.001, 0.5);
                tankDrive.Intake1.setPower(0);
                tankDrive.Intake2.setPower(0);
                freeze();

                movethForward(-14, 1.5, 0.003, 0.5); //Delivering to building zone
                turnethDirection(90, 0.5, 0.2, 0.003, 3);
                movethForward(40, 3, 0.003, 0.5);
                tankDrive.Intake1.setPower(-1.0);
                tankDrive.Intake2.setPower(-1.0);
                sleep(3000);
                tankDrive.Intake1.setPower(0);
                tankDrive.Intake2.setPower(0);
                movethForward(-85, 5, 0.003, 0.7); //Slam into wall

                movethForward(10, 1.5, 0.003, 0.7); //Collecting skystone
                turnOneWheelDirection(-90, 0.5, 0.2, 0.003, 3);
                tankDrive.RightNugget.setPosition(tankDrive.ROUT);
                tankDrive.LeftNugget.setPosition(tankDrive.LOUT);
                movethForward(34, 2.5, 0.001,0.5);
                tankDrive.Intake1.setPower(1.0);
                tankDrive.Intake2.setPower(1.0);
                tankDrive.RightNugget.setPosition(tankDrive.RIN);
                tankDrive.LeftNugget.setPosition(tankDrive.LIN);
                movethForward(5, 1.2, 0.001, 0.5);
                tankDrive.Intake1.setPower(0);
                tankDrive.Intake2.setPower(0);
                freeze();

                movethForward(-14, 1.5, 0.003, 0.5); //Going back
                turnethDirection(90, 0.5, 0.2, 0.003, 3);
                movethForward(80, 5, 0.003, 0.5); // Delivering to building zone
                tankDrive.Intake1.setPower(-1.0);
                tankDrive.Intake2.setPower(-1.0);
                sleep(3000);
                tankDrive.Intake1.setPower(0);
                tankDrive.Intake2.setPower(0);
                movethForward(-5, 1, 0.001, 0.4);
            }
            else if (skystonePosition == 3)
            {
                movethForward(3, 0.5, 0.001, 0.3);
                turnethDirection(-55, 0.5, 0.3, 0.003, 3);
                turnOneWheelDirection(55, 0.5, 0.3, 0.003, 3);
                movethForward(13, 2.5, 0.001, 0.8);
                freeze();
                turnethDirection(-15, 0.5, 0.3, 0.003, 3);
                runIntake(-0.75);
                movethForward(16, 2.5, 0.001, 0.6);
                sleep(1000);
                runIntake(0);
                movethForward(-30, 2.5, 0.001, 0.4);
                runIntake(0.5);
                sleep(110);
                runIntake(0);
                freeze();
                turnethDirection(105,0.35, 0.3, 0.005, 3);

            }
            else
            {
                telemetry.addData("ERROR", "SkyStone Not Found.");
                telemetry.update();
                freeze();
                break;
            }

            //skystonePosition = 0;
            runIntake(-1);
            movethForward(-40, 2, 0.001, 0.8, 0.4);
            freeze();
            runIntake(0);
            movethForward(15, 2, 0.001, 0.4);
            break;
            /*claw(true);
            movethForward(85, 2.5, 0.001, 0.6);
            turnethDirection(90,0.4, 0.25, 0.005, 4);
            movethForward(30, 2.5, 0.001, 0.6);
            sleep(500);
            claw(false);
            freeze();
            sleep(2000);
            movethForward(-10, 2.5, 0.001, 0.6);
            turnOneWheelDirection(-90, 0.5, 0.3, 0.005, 4);
            freeze();
            sleep(1000);
            claw(true);
            movethForward(-40, 2.5, 0.005, 0.6);
            freeze();

             */
        }
        freeze();
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
        int errorMargin = 10; //Amount of error in ticks we are willing to accept
        double powerFloor = 0.19; //Minimum power
        double powerCeiling = power; //Maximum power

        telemetry.addData("ticks", encoderTicks);
        telemetry.update();
        while (Math.abs(tankDrive.getAverageEncoder('l') + tankDrive.getAverageEncoder('r') - encoderTicks * 2) > errorMargin && clock.seconds() < t && opModeIsActive())
        {
            //double leftpower = Math.abs(encoderTicks - tank.getAverageEncoder('l')) * kp;
            double leftpower = sigmoid(encoderTicks - tankDrive.getAverageEncoder('l'), powerCeiling, powerFloor, 800, kp * 4);
            leftpower = Math.max(leftpower, powerFloor);
            leftpower = Math.min(leftpower, powerCeiling);
            if (encoderTicks - tankDrive.getAverageEncoder('l') < 0) leftpower *= -1;
            tankDrive.LM0.setPower(leftpower);
            tankDrive.LM1.setPower(leftpower);
            telemetry.addData("LeftPower", leftpower);

            //double rightpower = Math.abs(encoderTicks - tank.getAverageEncoder('r')) * kp;
            double rightpower = leftpower;
            tankDrive.RM0.setPower(rightpower);
            tankDrive.RM1.setPower(rightpower);

            telemetry.addData("RightPower", rightpower);
            telemetry.addData("Left Encoder", tankDrive.getAverageEncoder('l'));
            telemetry.addData("Right Encoder", tankDrive.getAverageEncoder('r'));
            telemetry.addData("ticks", encoderTicks);
            telemetry.addData("Current L", tankDrive.getAverageEncoder('l'));
            telemetry.addData("Current R", tankDrive.getAverageEncoder('l'));
            telemetry.addData("kP", kp);
            telemetry.update();
        }
        freeze();
        turnethDirection(0, 0.35, 0.2, 0.005, 3);
    }


    public void movethForward(double inches, double t, double kp, double ceil, double floor)
    {
        tankDrive.resetEncoders();
        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        double encoderTicks = inches / MOTOR_TO_INCHES * NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION; // target number of encoder tick
        int errorMargin = 10; //Amount of error in ticks we are willing to accept
        double powerFloor = floor; //Minimum power
        double powerCeiling = ceil; //Maximum power

        telemetry.addData("ticks", encoderTicks);
        telemetry.update();
        while (Math.abs(tankDrive.getAverageEncoder('l') + tankDrive.getAverageEncoder('r') - encoderTicks * 2) > errorMargin && clock.seconds() < t && opModeIsActive())
        {

            //double leftpower = Math.abs(encoderTicks - tank.getAverageEncoder('l')) * kp;
            double leftpower = sigmoid(encoderTicks - tankDrive.getAverageEncoder('l'), powerCeiling, powerFloor, 800, kp * 4);
            leftpower = Math.max(leftpower, powerFloor);
            leftpower = Math.min(leftpower, powerCeiling);
            if (encoderTicks - tankDrive.getAverageEncoder('l') < 0) leftpower *= -1;
            tankDrive.LM0.setPower(leftpower);
            tankDrive.LM1.setPower(leftpower);
            telemetry.addData("LeftPower", leftpower);

            //double rightpower = Math.abs(encoderTicks - tank.getAverageEncoder('r')) * kp;
            double rightpower = leftpower;
            tankDrive.RM0.setPower(rightpower);
            tankDrive.RM1.setPower(rightpower);

            telemetry.addData("Left Encoder", tankDrive.getAverageEncoder('l'));
            telemetry.addData("Right Encoder", tankDrive.getAverageEncoder('r'));
            telemetry.addData("ticks", encoderTicks);
            telemetry.addData("Current L", tankDrive.getAverageEncoder('l'));
            telemetry.addData("Current R", tankDrive.getAverageEncoder('l'));
            telemetry.addData("kP", kp);
            telemetry.update();
        }
        freeze();

        turnethDirection(0, 0.35, 0.2, 0.005, 3);
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

    private void turnOneWheelDirection(int initial, double powerCeiling, double powerFloor, double kp, double t) {

        lastAngles = imu.getAngularOrientation();
        double currentAngle = lastAngles.firstAngle;
        ElapsedTime clock = new ElapsedTime();
        clock.reset();
        globalAngle += initial;
        if (globalAngle > 180)
            globalAngle -= 360;
        if (globalAngle < -180)
            globalAngle += 360;
        double leftPower, rightPower;

        // restart imu movement tracking.
        //resetAngle();

        telemetry.addData("currentAngle", currentAngle);
        telemetry.update();
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // set power to rotate.

        // rotate until turn is completed.
        while (clock.seconds() < t && Math.abs(globalAngle - imu.getAngularOrientation().firstAngle) > 3 && opModeIsActive()) {
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

            //tankDrive.LM0.setPower(leftPower);
            //tankDrive.LM1.setPower(leftPower);
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
            tankDrive.LeftFang.setPosition(1); //true = up
            tankDrive.RightFang.setPosition(0);
        }
        else {
            tankDrive.LeftFang.setPosition(0); //false = down
            tankDrive.RightFang.setPosition(1);
        }
    }
}
