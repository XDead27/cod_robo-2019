package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Object_Detection_Back", group = "Vuforia")
@Disabled

public final class Object_Detection_Back extends Autonomous_Mode {

    @Override
    protected void runOperations() {

        FrontCamera = false;
        TestPosition();

    }

    void TestPosition(){
        MineralPosition now = Position();
        while(opModeIsActive()){
            telemetry.addData("position : " , now);
            telemetry.update();
            idle();
        }
    }

    @Override
    protected void endOperations() {


    }

}
