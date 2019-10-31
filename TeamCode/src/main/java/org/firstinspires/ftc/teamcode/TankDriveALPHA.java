package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class TankDriveALPHA
{
    public DcMotor  LM0  = null;
    public DcMotor  LM1  = null;
    public DcMotor  RM0  = null;
    public DcMotor  RM1  = null;

    public DcMotor  Intake1  = null;
    public DcMotor  Intake2 = null;

    HardwareMap hwMap =  null;

    public TankDriveALPHA(){
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        LM0 = hwMap.get(DcMotor.class, "LM0");
        LM1 = hwMap.get(DcMotor.class, "LM1");
        RM0 = hwMap.get(DcMotor.class, "RM0");
        RM1 = hwMap.get(DcMotor.class, "RM1");

        Intake1 = hwMap.get(DcMotor.class, "Intake1");
        Intake2 = hwMap.get(DcMotor.class, "Intake2");

        LM0.setDirection(DcMotor.Direction.REVERSE);
        LM1.setDirection(DcMotor.Direction.REVERSE);
        RM0.setDirection(DcMotor.Direction.FORWARD);
        RM1.setDirection(DcMotor.Direction.FORWARD);
        Intake1.setDirection(DcMotor.Direction.REVERSE);
        Intake2.setDirection(DcMotor.Direction.FORWARD);

        LM0.setPower(0);
        LM1.setPower(0);
        RM0.setPower(0);
        RM1.setPower(0);
        Intake1.setPower(0);
        Intake2.setPower(0);

        LM0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RM0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LM0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RM0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getEncoderAvg(char c) {
        switch (c)
        {
            case 'l': return (Math.abs(LM0.getCurrentPosition()) + Math.abs(LM1.getCurrentPosition())) / 2.0;
            case 'r': return (Math.abs(RM0.getCurrentPosition()) + Math.abs(RM1.getCurrentPosition())) / 2.0;
        }
        return Double.NaN;
    }

    public double getEncoderAvg() {
        return (Math.abs(LM0.getCurrentPosition()) + Math.abs(LM1.getCurrentPosition()) +
                Math.abs(RM0.getCurrentPosition()) + Math.abs(RM1.getCurrentPosition())) / 4.0;
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

