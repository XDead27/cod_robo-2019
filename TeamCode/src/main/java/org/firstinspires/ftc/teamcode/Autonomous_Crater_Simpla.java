package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous_Crater_Simpla", group = "Autonomous")

public final class Autonomous_Crater_Simpla extends Autonomous_Mode {

    @Override
    protected void runOperations() {

        //let the robot down
        LiftDown();

        //move left and forward a little
        WalkEncoder(15*TICKS_PER_CM , 0.5 , 90);
        WalkEncoder(30*TICKS_PER_CM , 0.5 , 0);

        //see where the cube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        //TODO : go the cube and push it
        if (now == LEFT){
            //strafa de 30 la 45
            //mers in fata 20
            //ers in spate 20
            //strafa de -30 la 45
            //strafa de 15 la -90

            //acum este in centru la aprox 20 de cm de ob din mijloc
        }
        else if (now == MIDDLE){
            //strafa de 30 la -45
            //mers in fata 20
            //mers in spate 40

            //acum este in centru la aprox 20 de cm de ob din mijloc
        }
        else if (now == RIGHT){



        }

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
