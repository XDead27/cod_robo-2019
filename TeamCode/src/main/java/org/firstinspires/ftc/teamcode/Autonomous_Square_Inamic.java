package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Square_Inamic", group = "Autonomous")

public final class Autonomous_Square_Inamic extends Autonomous_Mode {

    @Override
    protected void runOperations() {

        //let the robot down
        LiftDown();

        //move left and forward a little
        WalkEncoder(10*TICKS_PER_CM , 0.5 , 90);
        WalkEncoder(10*TICKS_PER_CM , 0.5 , 0);

        //see where the cube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        //TODO : go the cube and push it

        //TODO : go to the square

        //let the team marker down
        PlantTeamMarker();

        //TODO : go to the enemy crater

        ExtendSlidingSystem();
        LiftUp();

        //try to capture objects until the end
        GetObjects();



    }

    @Override
    protected void endOperations() {

    }

}
