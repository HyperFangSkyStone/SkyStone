package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class TankDriveHardware
{

    public DcMotor  LM0  = null;
    public DcMotor  LM1  = null;
    public DcMotor  RM0  = null;
    public DcMotor  RM1  = null;


    HardwareMap hwMap =  null;

    public TankDriveHardware(){
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        LM0 = hwMap.get(DcMotor.class, "LM0");
        LM1 = hwMap.get(DcMotor.class, "LM1");
        RM0 = hwMap.get(DcMotor.class, "RM0");
        RM1 = hwMap.get(DcMotor.class, "RM1");


        LM0.setDirection(DcMotor.Direction.FORWARD);
        LM1.setDirection(DcMotor.Direction.FORWARD);
        RM0.setDirection(DcMotor.Direction.REVERSE);
        RM1.setDirection(DcMotor.Direction.REVERSE);


        LM0.setPower(0);
        LM1.setPower(0);
        RM0.setPower(0);
        RM1.setPower(0);


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

