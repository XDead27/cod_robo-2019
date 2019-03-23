package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous_Square_Complex", group = "Autonomous")
@Disabled

public final class Autonomous_Square_Complex extends Autonomous_Mode {

    @Override
    protected void initialise(boolean bIsDriver){
        super.initialise(bIsDriver);
        telemetry.addData("waiting for start " , "");
        telemetry.update();
    }

    @Override
    protected void runOperations() {

        //let the robot down
        LiftDown();

        //calibrate gyro
        CalibrateGyro();

        //move left
        MoveToUnlatch();

        //see where the cube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        //choose cube
        ChooseCube(now);

        //TODO : TESTAT DE AICI IN JOS

        /*//let team marker
        MoveSlidersEncoder(200 , 0.5);
        ExtendSlidingSystem();
        PlantTeamMarker();
        RetractSlidingSystem();

        GoBackAndTurn(false, now);

        WalkObstacleAndRangeNORMAL(15 , false , 0.4);

        AlignWithWall();

        Rotate(-90);

        WalkObstacleAndRangeNORMAL(150 , true , 0.4);

        ParkAtCrater();*/

    }

    @Override
    protected void endOperations() {

    }
}
