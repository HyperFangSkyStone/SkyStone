package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;

public class absEncHW {

    public PWMOutput encoder = null;
    HardwareMap hwMap           =  null;


    public absEncHW()
    {

    };

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        encoder = hwMap.get(PWMOutput.class , "encoder");
    }
}
