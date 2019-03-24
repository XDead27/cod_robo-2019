package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous_Square_Complex", group = "Autonomous")
//@Disabled

public final class Autonomous_Square_Complex extends Autonomous_Mode {

    @Override
    protected void initialise(boolean bIsDriver) {
        super.initialise(bIsDriver);

        SteadyGlisiere();
    }

    @Override
    protected void runOperations() {

        //let the robot down
        LiftDown();

        //move left
        MoveToUnlatch();

        //Lift the sliders down to make the robot steadier
        MoveSlidersEncoder(1000 , 0.8);

        //see where the cube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        //choose cube
        ChooseCube(now);

        //moves slightly further from the other minerals then turns towards the crater/square
        AfterChooseMoveSequence(false, now, Direction.NORMAL);
        AfterChooseMoveSequence(false, now, Direction.REVERSE);

        if ( now == RIGHT ) {
            LetTeamMarker(false);
            WalkEncoder(100, 0.8, 180);
        }
        else {
            GoBackToCenter(now);
            ComplexCommute(false, now);
        }
    }

    @Override
    protected void endOperations() {

    }
}
