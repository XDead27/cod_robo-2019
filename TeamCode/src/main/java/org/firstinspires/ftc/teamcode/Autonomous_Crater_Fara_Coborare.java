package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous_Crater_Fara_Coborare", group = "Autonomous")

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

        //choose cube
        ChooseCube(now);

        GoBackAndTurn();

        WalkToWall();

        AlignWithWall();

        //ContinuareCrater(now);

        //park
        //ParkAtCrater();
    }

    @Override
    protected void endOperations() {

    }

}
