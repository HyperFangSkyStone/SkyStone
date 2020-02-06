package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;


@TeleOp(name="TankDrivePIDTeleop", group="Sans Sans")
@Disabled
public class TankDrivePIDTeleop extends LinearOpMode {
    public final double WHEEL_DIAMETER = 90; //Wheel diameter in mm
    public final int MOTOR_GEAR_TEETH = 26; //# of teeth on the motor gear
    public final int WHEEL_GEAR_TEETH = 20; //# of teeth on the wheel gear
    public final double GEAR_RATIO = (MOTOR_GEAR_TEETH + 0.0) / WHEEL_GEAR_TEETH; //For every full turn of the motor, the wheel turns this many rotations.
    public final double MM_TO_INCHES =  25.4;
    public final double MOTOR_TO_INCHES = GEAR_RATIO * WHEEL_DIAMETER * Math.PI / MM_TO_INCHES; //For every full turn of both motors, the wheel moves forward this many inches
    public final double NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION = 537.6;

    TankDriveHardware dsModule = new TankDriveHardware();
    ElapsedTime time = new ElapsedTime();
    ArrayList<Double> timeRecord = new ArrayList<>();
    BNO055IMU imu;
    double globalAngle;
    Orientation lastAngles = new Orientation();


    PIDController PID = new PIDController(0, 0, 0);


    @Override
    public void runOpMode() throws InterruptedException {
        dsModule.init(hardwareMap);

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

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        /*while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }*/

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("Number of inches per motor rotation", MOTOR_TO_INCHES);
        telemetry.update();

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

            if (gamepad1.a)
                goethEncoderTicks(1440, 5);
            else if (gamepad1.b)
                goethEncoderTicks(1000, 5);
            else if (gamepad1.x)
                goethEncoderTicks(360, 5);
            /*if(gamepad1.y) {
                telemetry.addData("Right Encoder Average", dsModule.getAverageEncoder('r'));
                telemetry.update();
            }
            dsModule.LM0.setPower(0);
            dsModule.LM1.setPower(0);
            dsModule.RM0.setPower(0);
            dsModule.RM1.setPower(0);*/
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
        dsModule.LM0.setPower(0);
        dsModule.LM1.setPower(0);
        dsModule.RM0.setPower(0);
        dsModule.RM1.setPower(0);
    }

    public void movethForward(double inches, double t)
    {
        dsModule.resetEncoders();
        time.reset();
        double encoderTicks = inches / MOTOR_TO_INCHES * NUMBER_OF_ENCODER_TICKS_PER_REVOLUTION; // target number of encoder ticks
        double kp = 0.002;
        int errorMargin = 5; //Amount of error in ticks we are willing to accept
        double powerFloor = 0.1; //Minimum power
        double powerCeiling = 0.2; //Maximum power

        telemetry.addData("Moveth Forward is moving forward at ", inches + " inches and ");
        telemetry.update();
        while (Math.abs(dsModule.getAverageEncoder('l') + dsModule.getAverageEncoder('r') - encoderTicks * 2) > errorMargin && time.seconds() < t)
        {
            if (Math.abs(encoderTicks - dsModule.getAverageEncoder('l')) >= errorMargin)
            {
                double leftpower = Math.abs(encoderTicks - dsModule.getAverageEncoder('l')) * kp;
                leftpower = Math.max(leftpower, powerFloor);
                leftpower = Math.min(leftpower, powerCeiling);
                if (encoderTicks - dsModule.getAverageEncoder('l') < 0) leftpower *= -1;
                dsModule.LM0.setPower(leftpower);
                dsModule.LM1.setPower(leftpower);
                telemetry.addData("LeftPower", leftpower);
            }
            else {
                dsModule.LM0.setPower(0);
                dsModule.LM1.setPower(0);
            }

            if (Math.abs(encoderTicks - dsModule.getAverageEncoder('r')) >= errorMargin)
            {
                double rightpower = Math.abs(encoderTicks - dsModule.getAverageEncoder('r')) * kp;
                rightpower = Math.max(rightpower, powerFloor);
                rightpower = Math.min(rightpower, powerCeiling);
                if (encoderTicks - dsModule.getAverageEncoder('r') < 0) rightpower *= -1;
                dsModule.RM0.setPower(rightpower);
                dsModule.RM1.setPower(rightpower);
                telemetry.addData("RightPower", rightpower);
            }
            else {
                dsModule.RM0.setPower(0);
                dsModule.RM1.setPower(0);
            }
            telemetry.addData("Left Encoder", dsModule.getAverageEncoder('l'));
            telemetry.addData("Right Encoder", dsModule.getAverageEncoder('r'));
            telemetry.update();

        }
    }

    public void goethEncoderTicks(double encoderTicks, double t)
    {
        dsModule.resetEncoders();
        time.reset();
        double kp = 0.002;
        int errorMargin = 20; //Amount of error in ticks we are willing to accept
        double powerFloor = 0.2; //Minimum power
        double powerCeiling = 0.8; //Maximum power

        while ((dsModule.getAverageEncoder('l') + dsModule.getAverageEncoder('r') < encoderTicks * 2) && time.seconds() < t)
        {
            if (Math.abs(encoderTicks - dsModule.getAverageEncoder('l')) >= errorMargin)
            {
                double leftpower = Math.abs(encoderTicks - dsModule.getAverageEncoder('l')) * kp;
                leftpower = Math.max(leftpower, powerFloor);
                leftpower = Math.min(leftpower, powerCeiling);
                if (encoderTicks - dsModule.getAverageEncoder('l') < 0) leftpower *= -1;
                dsModule.LM0.setPower(leftpower);
                dsModule.LM1.setPower(leftpower);
                telemetry.addData("LeftPower", leftpower);
            }
            else {
                dsModule.LM0.setPower(0);
                dsModule.LM1.setPower(0);
            }

            if (Math.abs(encoderTicks - dsModule.getAverageEncoder('r')) >= errorMargin)
            {
                double rightpower = Math.abs(encoderTicks - dsModule.getAverageEncoder('r')) * kp;
                rightpower = Math.max(rightpower, powerFloor);
                rightpower = Math.min(rightpower, powerCeiling);
                if (encoderTicks - dsModule.getAverageEncoder('r') < 0) rightpower *= -1;
                dsModule.RM0.setPower(rightpower);
                dsModule.RM1.setPower(rightpower);
                telemetry.addData("RightPower", rightpower);
            }
            else {
                dsModule.RM0.setPower(0);
                dsModule.RM1.setPower(0);
            }
            telemetry.addData("Left Encoder", dsModule.getAverageEncoder('l'));
            telemetry.addData("Right Encoder", dsModule.getAverageEncoder('r'));
            telemetry.update();

        }
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void turnethDirection(int degrees, double power) //NOT FUNCTIONAL; helper methods require testing
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        dsModule.LM0.setPower(leftPower);
        dsModule.LM1.setPower(leftPower);
        dsModule.RM0.setPower(rightPower);
        dsModule.RM1.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        dsModule.LM0.setPower(0);
        dsModule.LM1.setPower(0);
        dsModule.RM0.setPower(0);
        dsModule.RM1.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }


    // THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!!
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.


        // THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!!
        // THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!!
        // THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!!
        // THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!!
        // THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!!
        // THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!!
        // THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!!
        // THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!!
        // THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!! THIS NEEDS TO BE DETERMINED EXPERIMENTALLY!!!!!!!!!!!

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
}
