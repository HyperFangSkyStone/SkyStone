package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.TankDriveHardware;

import java.util.ArrayList;


@Autonomous(name="3inchBackwards", group="chimp")
//@Disabled
public class ONeinf extends LinearOpMode {
    public final double WHEEL_DIAMETER = 90; //Wheel diameter in mm
    public final int MOTOR_GEAR_TEETH = 26; //# of teeth on the motor gear
    public final int WHEEL_GEAR_TEETH = 20; //# of teeth on the wheel gear
    public final double GEAR_RATIO = (MOTOR_GEAR_TEETH + 0.0) / WHEEL_GEAR_TEETH; //For every full turn of the motor, the wheel turns this many rotations.
    public final double MM_TO_INCHES =  25.4;
    public final double MOTOR_TO_INCHES = GEAR_RATIO * WHEEL_DIAMETER * Math.PI / MM_TO_INCHES; //For every full turn of both motors, the wheel moves forward this many inches
    public final double NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION = 537.6;

    TankDriveHardware tank = new TankDriveHardware();
    ElapsedTime time = new ElapsedTime();
    ArrayList<Double> timeRecord = new ArrayList<>();
    BNO055IMU imu;
    double globalAngle;
    Orientation lastAngles = new Orientation();


    PIDController PID = new PIDController(0, 0, 0);

    public static int skystonePosition = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        tank.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        waitForStart();
        movethForward(-3, 5, 0.3, 0.4);


    }


    //  ++++++ Helper Methods ++++++

    public void linearMovement(double inputPower, double t) {

        time.reset();

        while (time.seconds() < t) {
            tank.LM0.setPower(inputPower);
            tank.LM1.setPower(inputPower);
            tank.RM0.setPower(inputPower);
            tank.RM1.setPower(inputPower);
            telemetry.update();
        }
        tank.LM0.setPower(0);
        tank.LM1.setPower(0);
        tank.RM0.setPower(0);
        tank.RM1.setPower(0);
    }

    public double sigmoid(double error, double ceiling, double floor, double half, double stiff) {
        return floor + (ceiling - floor) / (1 + Math.pow(Math.E, stiff * (half - error)));
    }

    public void movethForward(double inches, double t, double kp, double power)
    {
        tank.resetEncoders();
        time.reset();
        double encoderTicks = inches / MOTOR_TO_INCHES * NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION; // target number of encoder tick
        int errorMargin = 5; //Amount of error in ticks we are willing to accept
        double powerFloor = 0.1; //Minimum power
        double powerCeiling = power; //Maximum power

        telemetry.addData("Moveth Forward is moving forward at ", inches + " inches and ");
        telemetry.update();
        while (Math.abs(tank.getAverageEncoder('l') + tank.getAverageEncoder('r') - encoderTicks * 2) > errorMargin && time.seconds() < t)
        {
            if (Math.abs(encoderTicks - tank.getAverageEncoder('l')) >= errorMargin)
            {
                //double leftpower = Math.abs(encoderTicks - tank.getAverageEncoder('l')) * kp;
                double leftpower = sigmoid(encoderTicks - tank.getAverageEncoder('l'), powerCeiling, powerFloor, 800, kp * 6);
                leftpower = Math.max(leftpower, powerFloor);
                leftpower = Math.min(leftpower, powerCeiling);
                if (encoderTicks - tank.getAverageEncoder('l') < 0) leftpower *= -1;
                tank.LM0.setPower(leftpower);
                tank.LM1.setPower(leftpower);
                telemetry.addData("LeftPower", leftpower);
            }
            else {
                tank.LM0.setPower(0);
                tank.LM1.setPower(0);
            }

            if (Math.abs(encoderTicks - tank.getAverageEncoder('r')) >= errorMargin)
            {
                //double rightpower = Math.abs(encoderTicks - tank.getAverageEncoder('r')) * kp;
                double rightpower = sigmoid(encoderTicks - tank.getAverageEncoder('r'), powerCeiling, powerFloor, 800, kp * 6);
                rightpower = Math.max(rightpower, powerFloor);
                rightpower = Math.min(rightpower, powerCeiling);
                if (encoderTicks - tank.getAverageEncoder('r') < 0) rightpower *= -1;
                tank.RM0.setPower(rightpower);
                tank.RM1.setPower(rightpower);
                telemetry.addData("RightPower", rightpower);
            }
            else {
                tank.RM0.setPower(0);
                tank.RM1.setPower(0);
            }
            telemetry.addData("Left Encoder", tank.getAverageEncoder('l'));
            telemetry.addData("Right Encoder", tank.getAverageEncoder('r'));
            telemetry.update();

        }
    }

    public void goethEncoderTicks(double encoderTicks, double t)
    {
        tank.resetEncoders();
        time.reset();
        double kp = 0.002;
        int errorMargin = 20; //Amount of error in ticks we are willing to accept
        double powerFloor = 0.2; //Minimum power
        double powerCeiling = 0.8; //Maximum power

        while ((tank.getAverageEncoder('l') + tank.getAverageEncoder('r') < encoderTicks * 2) && time.seconds() < t)
        {
            if (Math.abs(encoderTicks - tank.getAverageEncoder('l')) >= errorMargin)
            {
                double leftpower = Math.abs(encoderTicks - tank.getAverageEncoder('l')) * kp;
                leftpower = Math.max(leftpower, powerFloor);
                leftpower = Math.min(leftpower, powerCeiling);
                if (encoderTicks - tank.getAverageEncoder('l') < 0) leftpower *= -1;
                tank.LM0.setPower(leftpower);
                tank.LM1.setPower(leftpower);
                telemetry.addData("LeftPower", leftpower);
            }
            else {
                tank.LM0.setPower(0);
                tank.LM1.setPower(0);
            }

            if (Math.abs(encoderTicks - tank.getAverageEncoder('r')) >= errorMargin)
            {
                double rightpower = Math.abs(encoderTicks - tank.getAverageEncoder('r')) * kp;
                rightpower = Math.max(rightpower, powerFloor);
                rightpower = Math.min(rightpower, powerCeiling);
                if (encoderTicks - tank.getAverageEncoder('r') < 0) rightpower *= -1;
                tank.RM0.setPower(rightpower);
                tank.RM1.setPower(rightpower);
                telemetry.addData("RightPower", rightpower);
            }
            else {
                tank.RM0.setPower(0);
                tank.RM1.setPower(0);
            }
            telemetry.addData("Left Encoder", tank.getAverageEncoder('l'));
            telemetry.addData("Right Encoder", tank.getAverageEncoder('r'));
            telemetry.update();

        }
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * param degrees Degrees to turn, + is left - is right
     */
    private void turnethDirection(int initial, double powerCeiling, double powerFloor, double kp)
    {
        lastAngles = imu.getAngularOrientation();
        double currentAngle = lastAngles.firstAngle;
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
        while (Math.abs(globalAngle - imu.getAngularOrientation().firstAngle) > 3)
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

            tank.LM0.setPower(leftPower);
            tank.LM1.setPower(leftPower);
            tank.RM0.setPower(rightPower);
            tank.RM1.setPower(rightPower);
        }

        // turn the motors off.
        tank.LM0.setPower(0);
        tank.LM1.setPower(0);
        tank.RM0.setPower(0);
        tank.RM1.setPower(0);

        // wait for rotation to stop.
        sleep(1000);
    }

    private double getAngle()
    {
        lastAngles = imu.getAngularOrientation();
        return lastAngles.firstAngle;
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
}
