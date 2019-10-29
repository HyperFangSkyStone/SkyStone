package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@SuppressWarnings("ALL")
@Autonomous(name="ApeDriveAuto", group="Obese Chimp")
//@Disabled
public class TankDriveAuto extends LinearOpMode {

    TankDriveHardware tankDrive = new TankDriveHardware();
    ElapsedTime clock = new ElapsedTime();


    PIDController linearPID = new PIDController(0, 0, 0);
    PIDController turningPID = new PIDController(0, 0, 0);

    public static int skystonePosition = 0;
    public static double distanceTimeIndex = 0; //time expected per inch
    public static double angleTimeIndex = 0; //time expected per degree

    @Override
    public void runOpMode() throws InterruptedException {

        tankDrive.init(hardwareMap);
        //waitForStart();

        VisionBitMapping vbm = new VisionBitMapping(this);

        while (!isStarted())
        {
            skystonePosition = vbm.skyStonePos();

            if (skystonePosition == 0) {
                telemetry.addData("ERROR", "SkyStone Not Found.");
                telemetry.update();
            }
        }

        while(isStarted())
        {
            if (skystonePosition == 1)
            {/*move to position 1*/}
            else if (skystonePosition == 2)
            {/*move to position 2*/}
            else if (skystonePosition == 3)
            {/*move to position 3*/}
            else
            {
                telemetry.addData("ERROR", "SkyStone Not Found.");
                telemetry.update();
                sleep(3);
                break;
            }




        }
    }


    public void runMotor(double leftInput, double rightInput)
    {
        tankDrive.LM0.setPower(leftInput);
        tankDrive.LM1.setPower(leftInput);
        tankDrive.RM0.setPower(rightInput);
        tankDrive.RM1.setPower(rightInput);
    }
    public void freeze()
    {
        tankDrive.LM0.setPower(0);
        tankDrive.LM1.setPower(0);
        tankDrive.RM0.setPower(0);
        tankDrive.RM1.setPower(0);
    }

    public void linearMovement(double distance)
    {
        double conversionIndex = 0;
        double timeFrame = distance * distanceTimeIndex;
        double power = 0;
        clock.reset();
        while (clock.seconds() < timeFrame /* && distance not reached*/ )
        {
            power = linearPID.PIDOutput(0,0,0,0);
            runMotor(power, power);
        }
    }

    public void turnToAngle(double angle)
    {

    }
}
