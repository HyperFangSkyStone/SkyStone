package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="Gateway Test", group="1")
//@Disabled
public class GatewayTest extends LinearOpMode {

    TankDriveALPHA tankDrive = new TankDriveALPHA();
    ElapsedTime clock = new ElapsedTime();

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

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("RightGate:", tankDrive.RightGate.getPosition());
            telemetry.addData("LeftGate:", tankDrive.LeftGate.getPosition());
            telemetry.addData("Pusher:", tankDrive.Pusher.getPosition());
            telemetry.addData("Vlad:", tankDrive.VladTheImpaler.getPosition());
            telemetry.update();
        }

    }
}
