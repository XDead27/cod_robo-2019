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

        MotorGlisieraL.setPower(-0.1);
        MotorGlisieraR.setPower(-0.1);
    }

    @Override
    protected void runOperations() {

        //let the robot down

        LiftDown();
        sleep(100);

        CalibrateGyro();

        //move left
        WalkEncoder(17 , 0.5 , 90);

        //see where the cube is
        LiftPhoneUp();
        MineralPosition now = Position(2);
        LiftPhoneDown();

        //TODO : go the cube and push it
        if (now == LEFT){
            WalkEncoder(35 , 0.5 , 0);
            WalkEncoder(70 , 0.5 , 45);
            WalkEncoder(20 , 0.5 , 0);
        }
        else if (now == MIDDLE){
            WalkEncoder(35 , 0.5 , 0);
            WalkEncoder(35 , 0.5 , -45);
            WalkEncoder(20 , 0.5 , 0);
        }
        else if (now == RIGHT){
            WalkEncoder(65 , 0.5 , -45);
            WalkEncoder(35 , 0.5 , -90);
            WalkEncoder(20 , 0.5 , 0);
        }

        /*//extend sliding system and let it down in the crater
        ExtendSlidingSystem();
        LiftUp();

        //try to capture objects until the end
        GetObjects();*/
    }

    @Override
    protected void endOperations() {

    }

}
