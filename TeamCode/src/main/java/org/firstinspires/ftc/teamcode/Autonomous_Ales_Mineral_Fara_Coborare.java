package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous_Mode;
import org.firstinspires.ftc.teamcode.MineralPosition;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous_Ales_Mineral_Fara_Coborare", group = "Autonomous")

public final class Autonomous_Ales_Mineral_Fara_Coborare extends Autonomous_Mode {

    @Override
    protected void initialise(boolean bIsDriver) {
        super.initialise(bIsDriver);
    }

    @Override
    protected void runOperations() {

        //calibrate gyro
        CalibrateGyro();

        //move left
        WalkEncoder(8 , 0.5 , 90);

        //see where the cube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        //go the cube and push it
        if (now == LEFT){
            WalkEncoder(35 , 0.5 , 0);
            WalkEncoder(45 , 0.5 , 45);
            WalkEncoder(30 , 0.5 , 0);
        }
        else if (now == MIDDLE){
            WalkEncoder(30 , 0.5 , 0);
            WalkEncoder(45 , 0.5 , -45);
            WalkEncoder(30 , 0.5 , 0);
        }
        else if (now == RIGHT){
            WalkEncoder(55 , 0.5 , -45);
            WalkEncoder(50 , 0.5 , -90);
            WalkEncoder(30 , 0.5 , 0);
        }

    }

    @Override
    protected void endOperations() {

    }

}

