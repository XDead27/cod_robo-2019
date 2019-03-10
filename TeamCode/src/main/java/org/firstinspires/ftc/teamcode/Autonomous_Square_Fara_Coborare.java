package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous_Square_Fara_Coborare", group = "Autonomous")

public final class Autonomous_Square_Fara_Coborare extends Autonomous_Mode {

    @Override
    protected void initialise(boolean bIsDriver){
        super.initialise(bIsDriver);
    }

    @Override
    protected void runOperations() {

        //calibrate gyro
        CalibrateGyro();

        MoveSlidersEncoder(1000 , 0.5);

        //move left
        WalkEncoder(9 , 0.5 , 90);

        //see where the cube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        //go the cube and push it
        if (now == LEFT){
            WalkEncoder(20 , 0.5 , 0);
            WalkEncoder(40 , 0.5 , 45);
            WalkEncoder(50 , 0.5 , 0);
            Rotate(-135);
            WalkEncoder(65 , 0.5 , 90);
        }
        else if (now == MIDDLE){
            WalkEncoder(30 , 0.5 , 0);
            WalkEncoder(45 , 0.5 , -45);
            WalkEncoder(80 , 0.5 , 0);
            Rotate(-90);
        }
        else if (now == RIGHT){
            WalkEncoder(55 , 0.5 , -45);
            WalkEncoder(40 , 0.5 , -90);
            WalkEncoder(80 , 0.5 , 0);
            Rotate(-70);
            WalkEncoder( 30, 0.5, 90);
        }

        PlantTeamMarker();

    }

    @Override
    protected void endOperations() {

    }
}
