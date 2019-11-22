package org.firstinspires.ftc.teamcode;

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

    public DcMotor  Intake1  = null;
    public DcMotor  Intake2  = null;

    public Servo LServo = null;
    public Servo LServo2 = null;
    public Servo RServo = null;
    public Servo RServo2 = null;
    public Servo PServo1 = null;
    public Servo PServo2 = null;

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

        LServo = hwMap.get(Servo.class, "LServo");
        LServo2 = hwMap.get(Servo.class, "LServo2");
        RServo = hwMap.get(Servo.class, "RServo");
        RServo2 = hwMap.get(Servo.class, "RServo2");
        PServo1 = hwMap.get(Servo.class, "PServo1");
        PServo2 = hwMap.get(Servo.class, "PServo2");

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
}

