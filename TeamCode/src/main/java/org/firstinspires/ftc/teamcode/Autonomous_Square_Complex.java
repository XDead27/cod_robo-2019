package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous_Square_Complex", group = "Autonomous")
//@Disabled

public final class Autonomous_Square_Complex extends Autonomous_Mode {

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
                Rotate(45);
                WalkEncoder(30, 0.5, -90);
                WalkEncoder(20, 0.5, 0);
                break;
        }

        //let team marker
        MoveSlidersEncoder(300 , 0.5);
        ExtendSlidingSystem();
        PlantTeamMarker();
        RetractSlidingSystem();

        MoveSlidersEncoder(1500, 0.5);

        //Rotate(-GetAngle());

        switch(now){
            case LEFT:
//                WalkEncoder(10, 0.5, -90);
//                Rotate(180);
//                WalkEncoder(150, 0.7, 0);

                WalkEncoder(50, 0.5, 90);
                WalkEncoder(40, 0.5, 45);
                Rotate(-45);
                WalkEncoder(60, 0.5, 90);
                Rotate(135);
                WalkEncoder(20, 0.5, -90);
                WalkEncoder(40, 0.4, 0);
                //WalkEncoder(-20, 0.5, 0);
                Rotate(90);
                break;

            case MIDDLE:
                WalkEncoder(25, 0.5, 180);
                WalkEncoder(60, 0.5, 90);
                Rotate(135);
                WalkEncoder(20, 0.5, -90);
                WalkEncoder(40, 0.4, 0);
                //WalkEncoder(-20, 0.5, 0);
                Rotate(90);
                break;

            case RIGHT:
                WalkEncoder(10, 0.5, 90);
                Rotate(180);
                WalkEncoder(150, 0.7, 0);
                break;
        }

        ParkAtCrater();
    }

    @Override
    protected void endOperations() {

    }
}
