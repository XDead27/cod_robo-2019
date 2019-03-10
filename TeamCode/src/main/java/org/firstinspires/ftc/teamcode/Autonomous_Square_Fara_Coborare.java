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
        WalkEncoder(10 , 0.5 , 90);

        //see where the cube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        //choose cube
        ChooseCube(now);

        //let team marker
        LetTeamMarker(now);

    }

    @Override
    protected void endOperations() {

    }
}
