package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class absEncHW {

    public AnalogInput encoder = null;
    HardwareMap hwMap           =  null;


    public absEncHW()
    {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        encoder = hwMap.get(AnalogInput.class, "encoder");
    }
}
