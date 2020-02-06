package org.firstinspires.ftc.mercury.DISABLED;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.mercury.TankDriveALPHA;

@TeleOp(name="Servo MONKEY", group="Capuchin")
@Disabled
public class ServoMOnk extends LinearOpMode {

    TankDriveALPHA servo = new TankDriveALPHA();

    @Override
    public void runOpMode() throws InterruptedException
    {
        servo.init(hardwareMap);

        waitForStart();


        while(opModeIsActive())
        {

            if (gamepad1.a)
            {
                servo.LeftFang.setPosition(1);
                servo.RightFang.setPosition(0);
            }

            else if (gamepad1.x)
            {
                servo.LeftFang.setPosition(0);
                servo.RightFang.setPosition(1);
            }
        }

    }


}
