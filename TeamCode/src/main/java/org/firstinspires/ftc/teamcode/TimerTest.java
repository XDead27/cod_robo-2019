package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Timer Test", group = "Autonomous")
public final class TimerTest extends LinearOpMode {
//    @Override
//    protected void runOperations() {
//        long RTimer = 0;
//
//        while(opModeIsActive()){
//            RTimer += 1;
//        }
//    }
//
//    @Override
//    protected void endOperations() {
//
//    }


    @Override
    public void runOpMode() {
        waitForStart();

        long RTimer = 0, period = 10;
        while(opModeIsActive()){
            RTimer += period;
            telemetry.addData("Timer", RTimer);
            telemetry.update();

            sleep(period);
        }
    }
}
