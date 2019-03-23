package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous Square Simpla Fara Coborare", group = "Autonomous")

public final class Autonomous_Square_Fara_Coborare extends Autonomous_Mode {

    @Override
    protected void initialise(boolean bIsDriver){
        super.initialise(bIsDriver);

        //calibrate gyro
        CalibrateGyro();

        telemetry.addData("waiting for start " , "");
        telemetry.update();
    }

    @Override
    protected void runOperations() {

        LiftSlidersUpABit();

        //move left
        MoveToUnlatch();

        //see where the cube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        //choose cube
        ChooseCube(now);

        //moves slightly further from the other minerals then turns towards the crater/square
        switch(now){
            case LEFT:
                WalkEncoder(20, 0.5, 90);
                Rotate(-35);
                break;

            case MIDDLE:
                WalkEncoder(10, 0.5, 0);
                break;

            case RIGHT:
                WalkEncoder(20, 0.5, -90);
                Rotate(35);
                break;
        }

        //let team marker
        MoveSlidersEncoder(500 , 0.5);
        ExtendSlidingSystem(false);
        PlantTeamMarker();
        RetractSlidingSystem();

//        GoBackAndTurn(false, now);

//        WalkObstacleAndRangeNORMAL(15 , false , 0.4);

//        AlignWithWall();
//
//        Rotate(-90);
//
//        WalkObstacleAndRangeNORMAL(150 , true , 0.4);
//
//        ParkAtCrater();

    }

    @Override
    protected void endOperations() {

    }
}
