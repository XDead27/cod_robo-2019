package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Test", group = "Autonomous")

public final class Autonomous_Test extends Autonomous_Mode {

    @Override
    protected void runOperations() {
        WalkAtAngle(1, 45);

        while(opModeIsActive()){
            idle();
        }

        StopMotors();
    }

    @Override
    protected void endOperations() {

    }

}
