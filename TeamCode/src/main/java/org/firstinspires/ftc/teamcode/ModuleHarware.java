package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ModuleHarware
{

    public DcMotor  M0  = null;
    public DcMotor  M1  = null;


    HardwareMap hwMap =  null;

    public ModuleHarware(){
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        M0 = hwMap.get(DcMotor.class, "M0");
        M1 = hwMap.get(DcMotor.class, "M1");
        M0.setDirection(DcMotor.Direction.FORWARD);
        M1.setDirection(DcMotor.Direction.FORWARD);


        M0.setPower(0);
        M1.setPower(0);


        M0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        M0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}

