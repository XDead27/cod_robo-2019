package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name= "Autonomie_Crater_Complex_Fara_Coborare", group="Autonomous")
@Disabled


public final class Autonomous_Crater_Complex_Fara_Coborare extends Autonomous_Mode {

    @Override
    protected void initialise(boolean bIsDriver) {
        super.initialise(bIsDriver);

        //calibrate gyro
        CalibrateGyro();
    }

    @Override
    protected void runOperations() {

        LiftSlidersUpABit();

        MoveToUnlatch();

        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        ChooseCube(now);

        //TODO: de continuat de aici

        GoBackAndTurn(true, now);

        WalkToWall(5);
        //WalkObstacleAndRangeNORMAL(10 , false , 0.4);

        AlignWithWall(-1);

        Rotate(90);

        WalkEncoder(30, 0.5, 0);

        AlignWithWall(1);

        WalkToWall(50);

        /*//WalkToWall();
        WalkObstacleAndRangeNORMAL(15 , true , 0.4);

        PlantTeamMarker();

        Rotate(180);

        WalkObstacleAndRangeNORMAL(150 , true , 0.4);

        ParkAtCrater();*/

    }

    @Override
    protected void endOperations() {

    }
}
