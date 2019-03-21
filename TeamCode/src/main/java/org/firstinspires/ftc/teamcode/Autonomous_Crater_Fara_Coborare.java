package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous_Crater_Fara_Coborare", group = "Autonomous")

public final class Autonomous_Crater_Fara_Coborare extends Autonomous_Mode {

    @Override
    protected void initialise(boolean bIsDriver) {
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

        GoBackAndTurn();

        //TODO: TESTAT DE AICI IN JOS

        //WalkToWall();
        WalkObstacleAndRangeNORMAL(15 , false , 0.4);

        AlignWithWall();

        Rotate(90);

        //WalkToWall();
        WalkObstacleAndRangeNORMAL(15 , true , 0.4);

        PlantTeamMarker();

        Rotate(180);

        WalkObstacleAndRangeNORMAL(150 , true , 0.4);

        ParkAtCrater();
    }

    @Override
    protected void endOperations() {

    }

}
