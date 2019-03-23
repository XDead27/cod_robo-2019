package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Timer Test", group = "Autonomous")
@Disabled


public final class TimerTest extends LinearOpMode {
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
