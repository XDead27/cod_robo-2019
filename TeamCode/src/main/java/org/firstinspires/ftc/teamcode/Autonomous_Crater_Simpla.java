package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Crater_Simpla", group = "Autonomous")

public final class Autonomous_Crater_Simpla extends Autonomous_Mode {

    @Override
    protected void runOperations() {

        //let the robot down
        LiftDown();

        //move left and forward a little
        WalkEncoder(10*TICKS_PER_CM , 0.5 , 90);
        WalkEncoder(10*TICKS_PER_CM , 0.5 , 0);

        //see where the qube




    }

    @Override
    protected void endOperations() {

    }

}
