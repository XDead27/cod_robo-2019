package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous_Square_Simpla_Coborare", group = "Autonomous")

public final class Autonomous_Square_Simpla extends Autonomous_Mode {

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

        MoveSlidersEncoder(1000 , 0.5);


        //see where the cube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        //choose cube
        ChooseCube(now);

        //let team marker
        MoveSlidersEncoder(200 , 0.5);
        ExtendSlidingSystem();
        PlantTeamMarker();
        RetractSlidingSystem();

        GoBackAndTurn(false, now);

        //let team marker
        //LetTeamMarker(now);

        //TODO: vazut ce dracu fac in jos
        /*Rotate(-90);

        WalkObstacleAndRangeNORMAL(20, true, 0.8);*/

    }

    @Override
    protected void endOperations() {

    }
}
