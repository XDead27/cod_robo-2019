package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Autonomous_Crater_Simpla_Optimizare", group = "Autonomous")
@Disabled

public final class Autonomous_Crater_Simpla_Optimizare extends Autonomous_Mode {

    @Override
    protected void runOperations() {

        //let the robot down
        LiftDown();

        //move left and forward a little
        WalkEncoder(10*TICKS_PER_CM , 0.5 , 90);
        WalkEncoder(10*TICKS_PER_CM , 0.5 , 0);

        //see where the qube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        //TODO : go the qube and push it

        //TODO : go to the right of the crater

        //extend sliding system and let it down in the crater
        ExtendSlidingSystem();
        LiftUp();

        //try to capture objects until the end
        GetObjects();
    }

    @Override
    protected void endOperations() {

    }

}
