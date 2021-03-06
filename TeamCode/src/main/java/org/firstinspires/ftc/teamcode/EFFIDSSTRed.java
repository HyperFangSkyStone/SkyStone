package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressWarnings("ALL")
@Autonomous(name="EFFIDSST Red", group="red")
//@Disabled
public class EFFIDSSTRed extends LinearOpMode {

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


        tankDrive.init(hardwareMap);
        //waitForStart();

        VisionBitMapping vbm = new VisionBitMapping(this);

        tankDrive.RightNugget.setPosition(tankDrive.ROUT);
        tankDrive.LeftNugget.setPosition(tankDrive.LOUT);
        //tankDrive.LeftGate.setPosition(TankDriveALPHA.LEFT_GATE_UP_POS);
        tankDrive.RightGate.setPosition(TankDriveALPHA.RIGHT_GATE_UP_POS);
        tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_UP_POS);
        while (!isStarted())
        {
            //skystonePosition = 3;//vbm.skyStonePos('b');


            double avgx = vbm.avgX();
            if (avgx < TankDriveALPHA.RED_DIVIDER_ONE) {
                skystonePosition = 1;
            } else if (avgx < TankDriveALPHA.RED_DIVIDER_TWO) {
                skystonePosition = 2;
            } else {
                skystonePosition = 3;
            }


            telemetry.addData("Skystone Pos", skystonePosition);
            telemetry.update();
            if (skystonePosition == 0) {
                telemetry.addData("ERROR", "SkyStone Not Found.");
                telemetry.update();
            }
        }

        waitForStart();
        if(!isStopRequested())
        {
            collection1(skystonePosition);
            delivery1(skystonePosition);
            collection2(skystonePosition);
            delivery2(skystonePosition);
        }

        tankDrive.RightGate.setPosition(TankDriveALPHA.RIGHT_GATE_DOWN_POS);
        //tankDrive.LeftGate.setPosition(TankDriveALPHA.LEFT_GATE_DOWN_POS);
        tankDrive.resetEncoders();
        while (opModeIsActive())
            tankDrive.stayInPlace();

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

            double currentAngle = imu.getAngularOrientation().firstAngle;
            double raw = globalAngle - currentAngle;
            if (raw > 180)
                raw -= 360;
            if (raw < -180)
                raw += 360;
            double fudgeFactor = 1 - raw / 30;//1 + raw / 60;

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
        while (clock.seconds() < t && Math.abs(globalAngle - imu.getAngularOrientation().firstAngle) > 1 && opModeIsActive())
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

            telemetry.addData("power", leftPower);
            telemetry.update();

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
        sleep(500);
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
        while (clock.seconds() < t && Math.abs(globalAngle - imu.getAngularOrientation().firstAngle) > 1 && opModeIsActive())
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



            telemetry.addData("power", leftPower);
            telemetry.update();
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


    private void collection1(int skystonePosition) {
        switch(skystonePosition) {
            case 1:
                turnOneWheelDirection(8, 0.6, 0.35, 0.03, 3);
                runIntake(-1);
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_HOLD_POS);
                overshootLinearMovement(35,4);
                tankDrive.RightNugget.setPosition(tankDrive.RIN);
                tankDrive.LeftNugget.setPosition(tankDrive.LIN);
                sleep(400);
                break;



            case 2:
                turnOneWheelDirection(1, 0.6, 0.5, 0.008, 1); // Collect skystone
                runIntake(-1);
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_HOLD_POS);
                overshootLinearMovement(35,4);
                tankDrive.RightNugget.setPosition(tankDrive.RIN);
                tankDrive.LeftNugget.setPosition(tankDrive.LIN);
                sleep(600);
                break;



            case 3:
                turnOneWheelDirection(-6, 0.6, 0.5, 0.005, 2, 'l');
                runIntake(-1);
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_HOLD_POS);
                overshootLinearMovement(40,4);
                tankDrive.RightNugget.setPosition(tankDrive.RIN);
                tankDrive.LeftNugget.setPosition(tankDrive.LIN);
                sleep(400);
                break;
        }
    }

    private void delivery1(int skystonePosition) {
        switch(skystonePosition) {
            case 1:
                pidLinearMovement(-5, 1.2);
                runIntake(0);
                turnOneWheelDirection(77, 0.6, 0.4, 0.008, 5, 'l'); //3.5
                pidLinearMovement(-37, 3);
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_UNHOLD_POS);
                runIntake(-1);
                sleep(1000);
                runIntake(0);
                break;



            case 2:
                pidLinearMovement(-3, 2);
                runIntake(0);
                turnOneWheelDirection(88, 0.6, 0.4, 0.008, 3.5, 'l'); //3.5 // Deliver skystone
                pidLinearMovement(-35, 3);
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_UNHOLD_POS);
                runIntake(-1);
                sleep(1000);
                runIntake(0);
                break;



            case 3:
                pidLinearMovement(-7, 2);
                runIntake(0);
                turnOneWheelDirection(96, 0.6, 0.4, 0.008, 5, 'l'); //3.5
                pidLinearMovement(-25, 3);
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_UNHOLD_POS);
                runIntake(-1);
                sleep(1000);
                runIntake(0);
                break;
        }
    }

    private void collection2(int skystonePosition) {
        switch(skystonePosition) {
            case 1:
                pidLinearMovement(58, 6);
                tankDrive.RightNugget.setPosition(tankDrive.ROUT);
                tankDrive.LeftNugget.setPosition(tankDrive.LOUT);
                turnOneWheelDirection(-45, 0.6, 0.4, 0.008, 4, 'l'); //2.5
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_HOLD_POS);
                runIntake(-1);
                pidLinearMovement(18, 1.5);
                tankDrive.RightNugget.setPosition(tankDrive.RIN);
                tankDrive.LeftNugget.setPosition(tankDrive.LIN);
                sleep(1000);
                runIntake(0);
                break;



            case 2:
                pidLinearMovement(65, 6);
                tankDrive.RightNugget.setPosition(tankDrive.ROUT);
                tankDrive.LeftNugget.setPosition(tankDrive.LOUT);
                turnOneWheelDirection(-85, 0.6, 0.4, 0.008, 3.5, 'l'); //2.5
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_HOLD_POS);
                runIntake(-1);
                overshootLinearMovement(16, 3);
                tankDrive.RightNugget.setPosition(tankDrive.RIN);
                tankDrive.LeftNugget.setPosition(tankDrive.LIN);
                sleep(1000);
                runIntake(0);
                break;



            case 3:
                pidLinearMovement(55, 6);
                tankDrive.RightNugget.setPosition(tankDrive.ROUT);
                tankDrive.LeftNugget.setPosition(tankDrive.LOUT);
                turnOneWheelDirection(-90, 0.6, 0.4, 0.008, 5, 'l'); //2.5
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_HOLD_POS);
                runIntake(-1);
                overshootLinearMovement(15, 2);
                tankDrive.RightNugget.setPosition(tankDrive.RIN);
                tankDrive.LeftNugget.setPosition(tankDrive.LIN);
                sleep(1000);
                break;
        }
    }

    private void delivery2(int skystonePosition) {
        switch(skystonePosition) {
            case 1:
                pidLinearMovement(-12, 2);
                turnOneWheelDirection(45, 0.6, 0.4, 0.008, 4, 'l');
                pidLinearMovement(-58, 3);
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_UNHOLD_POS);
                runIntake(-1);
                sleep(1000);
                runIntake(0);
                pidLinearMovement(20, 3);
                //tankDrive.LeftGate.setPosition(TankDriveALPHA.LEFT_GATE_DOWN_POS);
                tankDrive.RightGate.setPosition(TankDriveALPHA.RIGHT_GATE_DOWN_POS);
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_UP_POS);
                break;



            case 2:
                pidLinearMovement(-6, 1.5);
                turnOneWheelDirection(81, 0.6, 0.4, 0.008, 3.5, 'l');
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_UNHOLD_POS);
                pidLinearMovement(-65, 6);
                runIntake(-1);
                sleep(1000);
                runIntake(0);
                pidLinearMovement(17, 3);
                //tankDrive.LeftGate.setPosition(TankDriveALPHA.LEFT_GATE_DOWN_POS);
                tankDrive.RightGate.setPosition(TankDriveALPHA.RIGHT_GATE_DOWN_POS);
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_UP_POS);
                break;



            case 3:
                runIntake(0);
                pidLinearMovement(-6, 1.5);
                turnOneWheelDirection(90, 0.6, 0.4, 0.008, 5, 'l'); //2.5
                //pidLinearMovement(-10, 1.5);
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_UNHOLD_POS);
                pidLinearMovement(-52, 3);
                runIntake(-1);
                sleep(1000);
                runIntake(0);
                pidLinearMovement(21, 2.3);
                tankDrive.Pusher.setPosition(TankDriveALPHA.PUSHER_UP_POS);
                //tankDrive.LeftGate.setPosition(TankDriveALPHA.LEFT_GATE_DOWN_POS);
                tankDrive.RightGate.setPosition(TankDriveALPHA.RIGHT_GATE_DOWN_POS);
                break;
        }
    }
}
