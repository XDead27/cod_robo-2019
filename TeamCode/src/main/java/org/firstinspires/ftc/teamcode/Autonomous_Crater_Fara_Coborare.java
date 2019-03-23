package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous Crater Simpla Fara Coborare", group = "Autonomous")

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

        telemetry.addData("now", now);
        telemetry.update();

        //choose cube
        ChooseCube(now);

        MoveSlidersEncoder(1000, 0.5);

        //moves slightly further from the other minerals then turns towards the crater/square
        switch(now){
            case LEFT:
                WalkEncoder(20, 0.5, 90);
                Rotate(-15);
                break;

            case MIDDLE:
                WalkEncoder(10, 0.5, 0);
                break;

            case RIGHT:
                WalkEncoder(20, 0.5, -90);
                Rotate(15);
                break;
        }

        ParkAtCrater();
    }

    @Override
    protected void endOperations() {

    }

}
