package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Test", group = "Autonomous")

public final class Autonomous_Test extends Autonomous_Mode {

    @Override
    protected void runOperations() {
        WalkEncoder(10 * 67, 0.4, 45);
        WalkEncoder(10 * 67, 0.4, 0);
    }

    @Override
    protected void endOperations() {


    }

}
