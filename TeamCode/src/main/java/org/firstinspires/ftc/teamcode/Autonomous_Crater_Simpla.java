package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Crater Simpla", group = "Autonomous")

public final class Autonomous_Crater_Simpla extends Autonomous_Mode {

    @Override
    protected void runOperations() {

        //let the robot down
        LiftDown();

        //move left
        MoveToUnlatch();

        //see where the cube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        MoveSlidersEncoder(1000, 0.5);

        //choose cube
        ChooseCube(now);

        //moves slightly further from the other minerals then turns towards the crater/square
        AfterChooseMoveSequence(true, now, Direction.NORMAL);

        ParkAtCrater();
    }

    @Override
    protected void endOperations() {

    }

}
