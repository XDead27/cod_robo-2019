package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous Square Simpla Cu Coborare", group = "Autonomous")

public final class Autonomous_Square_Simpla extends Autonomous_Mode {

    @Override
    protected void runOperations() {

        //let the robot down
        LiftDown();

        //move left
        MoveToUnlatch();

        //Lift the sliders down to make the robot steadier
        MoveSlidersEncoder(1000 , 0.5);

        //see where the cube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        //choose cube
        ChooseCube(now);

        //moves slightly further from the other minerals then turns towards the crater/square
        switch(now){
            case LEFT:
                Rotate(-45);
                WalkEncoder(20, 0.5, 90);
                WalkEncoder(20, 0.5, 0);
                break;

            case MIDDLE:
                WalkEncoder(10, 0.5, 0);
                break;

            case RIGHT:
                WalkEncoder(20, 0.5, -90);
                Rotate(35);
                WalkEncoder(20, 0.5, 0);
                break;
        }

        //let team marker
        MoveSlidersEncoder(500 , 0.5);
        ExtendSlidingSystem();
        PlantTeamMarker();
        RetractSlidingSystem();
    }

    @Override
    protected void endOperations() {

    }
}
