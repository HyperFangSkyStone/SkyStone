package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;


public class TankDriveALPHA
{
    public DcMotor  LM0  = null;
    public DcMotor  LM1  = null;
    public DcMotor  RM0  = null;
    public DcMotor  RM1  = null;
    public DcMotor Lift1 = null;
    public DcMotor Lift2 = null;

    public DcMotor Intake1  = null;
    public DcMotor Intake2  = null;

    public Servo LeftFang = null;
    public Servo LeftNugget = null;
    public Servo RightFang = null;
    public Servo RightNugget = null;
    public CRServo LeftBall = null; // Ball servos
    public CRServo RightBall = null;

    public Servo LeftClaw = null;
    public Servo RightClaw = null;
    public Servo PosClaw = null;

    public static final double LOUT = 0.0;
    public static final double LIN = 1.0;
    public static final double ROUT = 1.0;
    public static final double RIN = 0.0;

    HardwareMap hwMap =  null;

    public TankDriveALPHA(){
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        LM0 = hwMap.get(DcMotor.class, "LM0");
        LM1 = hwMap.get(DcMotor.class, "LM1");
        RM0 = hwMap.get(DcMotor.class, "RM0");
        RM1 = hwMap.get(DcMotor.class, "RM1");
        Lift1 = hwMap.get(DcMotor.class, "Lift1");
        Lift2 = hwMap.get(DcMotor.class, "Lift2");

        Intake1 = hwMap.get(DcMotor.class, "Intake1");
        Intake2 = hwMap.get(DcMotor.class, "Intake2");

        LeftFang = hwMap.get(Servo.class, "LeftFang");
        LeftNugget = hwMap.get(Servo.class, "LeftNugget");
        RightFang = hwMap.get(Servo.class, "RightFang");
        RightNugget = hwMap.get(Servo.class, "RightNugget");
        LeftBall = hwMap.get(CRServo.class, "LeftBall");
        RightBall = hwMap.get(CRServo.class, "RightBall");
        LeftClaw = hwMap.get(Servo.class, "LeftClaw");
        RightClaw = hwMap.get(Servo.class, "RightClaw");
        PosClaw = hwMap.get(Servo.class, "PosClaw");

        LM0.setDirection(DcMotor.Direction.REVERSE);
        LM1.setDirection(DcMotor.Direction.REVERSE);
        RM0.setDirection(DcMotor.Direction.FORWARD);
        RM1.setDirection(DcMotor.Direction.FORWARD);
        Lift1.setDirection(DcMotor.Direction.FORWARD);
        Lift2.setDirection(DcMotor.Direction.REVERSE);
        Intake1.setDirection(DcMotor.Direction.REVERSE);
        Intake2.setDirection(DcMotor.Direction.FORWARD);

        LM0.setPower(0);
        LM1.setPower(0);
        RM0.setPower(0);
        RM1.setPower(0);
        Lift1.setPower(0);
        Lift2.setPower(0);
        Intake1.setPower(0);
        Intake2.setPower(0);

        LM0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RM0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LM0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RM0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LM0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RM0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public double getEncoderAvg(char c) {
        switch (c)
        {
            case 'l': return (Math.abs(LM0.getCurrentPosition()) + Math.abs(LM1.getCurrentPosition())) / 2.0;
            case 'r': return (Math.abs(RM0.getCurrentPosition()) + Math.abs(RM1.getCurrentPosition())) / 2.0;
        }
        return Double.NaN;
    }
    public double getAverageEncoder(char c) {
        switch (c)
        {
            case 'l':
                if (LM0.getCurrentPosition() == 0)
                    return LM1.getCurrentPosition();
                else if (LM1.getCurrentPosition() == 0)
                    return LM0.getCurrentPosition();
                else return (Math.abs(LM0.getCurrentPosition()) + Math.abs(LM1.getCurrentPosition())) / 2.0;
            case 'r':
                if (RM0.getCurrentPosition() == 0)
                    return RM1.getCurrentPosition();
                else if (RM1.getCurrentPosition() == 0)
                    return RM0.getCurrentPosition();
                else return (Math.abs(RM0.getCurrentPosition()) + Math.abs(RM1.getCurrentPosition())) / 2.0;
        }
        return 0 / 0;
    }


    public double getEncoderAvg() {
        double output = 0;
        int encoderCount = 0;
        boolean[] encoderIsNotPluggedIn = new boolean[4];

        if (Math.abs(RM0.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(RM0.getCurrentPosition());
        } else encoderIsNotPluggedIn[0] = true;

        if (Math.abs(RM1.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(RM1.getCurrentPosition());
        } else encoderIsNotPluggedIn[1] = true;

        if (Math.abs(LM0.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(LM0.getCurrentPosition());
        } else encoderIsNotPluggedIn[2] = true;

        if (Math.abs(LM1.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(LM1.getCurrentPosition());
        } else encoderIsNotPluggedIn[3] = true;

        if (encoderCount == 0)
            return 0;
        else
            return output/encoderCount;
    }

    public double getEncoderAvg(Telemetry telemetry) {
        double output = 0;
        int encoderCount = 0;
        boolean[] encoderIsNotPluggedIn = new boolean[4];

        if (Math.abs(RM0.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(RM0.getCurrentPosition());
        } else encoderIsNotPluggedIn[0] = true;

        if (Math.abs(RM1.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(RM1.getCurrentPosition());
        } else encoderIsNotPluggedIn[1] = true;

        if (Math.abs(LM0.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(LM0.getCurrentPosition());
        } else encoderIsNotPluggedIn[2] = true;

        if (Math.abs(LM1.getCurrentPosition()) != 0) {
            encoderCount++;
            output += Math.abs(LM1.getCurrentPosition());
        } else encoderIsNotPluggedIn[3] = true;

        telemetry.addData("R0, R1, L0, L1", Arrays.toString(encoderIsNotPluggedIn));
        if (encoderCount == 0)
            return 0;
        else
            return output/encoderCount;
    }

    public void resetEncoders() {

        LM0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RM0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LM0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RM0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runMotor(double universalInput)
    {
        LM0.setPower(universalInput);
        LM1.setPower(universalInput);
        RM0.setPower(universalInput);
        RM1.setPower(universalInput);
    }

    public void runMotor(double leftInput, double rightInput)
    {
        LM0.setPower(leftInput);
        LM1.setPower(leftInput);
        RM0.setPower(rightInput);
        RM1.setPower(rightInput);
    }
    public void freeze() //completely stops drivetrain
    {
        LM0.setPower(0);
        LM1.setPower(0);
        RM0.setPower(0);
        RM1.setPower(0);
    }

    public void fang(boolean x) //foundation fangs manipulation
    {
        if (x) //fangs up
        {
            LeftFang.setPosition(1);
            RightFang.setPosition(0);
        }
        else //fangs down
        {
            LeftFang.setPosition(0);
            RightFang.setPosition(1);
        }
    }
}

