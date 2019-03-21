package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous_Square", group = "Autonomous")

public final class Autonomous_Square extends Autonomous_Mode {

    @Override
    protected void initialise(boolean bIsDriver){
        super.initialise(bIsDriver);
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

        //let team marker
        LetTeamMarker(now);

        //AICI IS PUSE DOAR CA SA APARA IN JURNAL PT CA TREBUIA SCOS INAINTE SA LE PUTEM FACE :^))))))))))))))))))
        Rotate(-90);

        WalkObstacleAndRangeNORMAL(20, true, 0.8);

    }

    @Override
    protected void endOperations() {

    }
}
