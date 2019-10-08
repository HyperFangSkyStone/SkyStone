package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class DriveTrainHardware
{

    public DcMotor  LeftM1  = null;
    public DcMotor  LeftM2  = null;
    public DcMotor  RightM1  = null;
    public DcMotor  RightM2  = null;


    HardwareMap hwMap           =  null;

    public DriveTrainHardware(){
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        LeftM1 = hwMap.get(DcMotor.class, "LeftM1");
        LeftM2 = hwMap.get(DcMotor.class, "LeftM2");
        RightM1 = hwMap.get(DcMotor.class, "RightM1");
        RightM2 = hwMap.get(DcMotor.class, "RightM2");
        LeftM1.setDirection(DcMotor.Direction.FORWARD);
        LeftM2.setDirection(DcMotor.Direction.FORWARD);
        RightM1.setDirection(DcMotor.Direction.FORWARD);
        RightM2.setDirection(DcMotor.Direction.FORWARD);


        LeftM1.setPower(0);
        LeftM2.setPower(0);
        RightM1.setPower(0);
        RightM2.setPower(0);


        LeftM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LeftM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}

