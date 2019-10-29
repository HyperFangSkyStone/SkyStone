package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PWMOutputController;
import com.qualcomm.robotcore.hardware.PWMOutputEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftandIntakeHW {

    public DcMotor LiftM1 = null;
    public DcMotor LiftM2 = null;
    public DcMotor IntakeM1 = null;
    public DcMotor IntakeM2 = null;
    public Servo Fang1 = null;
    public Servo Fang2 = null;

    HardwareMap hwMap          =  null;


    public LiftandIntakeHW()
    {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        LiftM1 = hwMap.get(DcMotor.class, "LiftM1");
        LiftM2 = hwMap.get(DcMotor.class, "LiftM2");
        IntakeM1 = hwMap.get(DcMotor.class, "IntakeM1");
        IntakeM2 = hwMap.get(DcMotor.class, "IntakeM2");
        Fang1 = hwMap.get(Servo.class, "Fang1");
        Fang2 = hwMap.get(Servo.class, "Fang2");


        LiftM1.setDirection(DcMotor.Direction.REVERSE);
        LiftM2.setDirection(DcMotor.Direction.FORWARD);
        IntakeM1.setDirection(DcMotor.Direction.REVERSE);
        IntakeM2.setDirection(DcMotor.Direction.FORWARD);


        LiftM1.setPower(0);
        LiftM2.setPower(0);
        IntakeM1.setPower(0);
        IntakeM2.setPower(0);


        LiftM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LiftM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
