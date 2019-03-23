package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous_Crater_Simpla", group = "Autonomous")

public final class Autonomous_Crater_Simpla extends Autonomous_Mode {

    @Override
    protected void initialise(boolean bIsDriver) {
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

        Rotate(-GetAngle());

        //see where the cube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        MoveSlidersEncoder(1000, 0.5);

        //choose cube
        ChooseCube(now);

        //moves slightly further from the other minerals then turns towards the crater/square
        switch(now){
            case LEFT:
                WalkEncoder(20, 0.5, 90);
                Rotate(-15);
                break;

            case MIDDLE:
                WalkEncoder(5, 0.5, 0);
                break;

            case RIGHT:
                WalkEncoder(20, 0.5, -90);
                Rotate(15);
                break;
        }

        //park
        ParkAtCrater();
    }

    @Override
    protected void endOperations() {

    }

}
