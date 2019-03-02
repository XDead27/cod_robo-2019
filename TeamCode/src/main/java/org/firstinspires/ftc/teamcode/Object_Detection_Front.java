package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Object_Detection_Front", group = "Vuforia")
@Disabled

public final class Object_Detection_Front extends Autonomous_Mode {

    @Override
    protected void runOperations() {

        FrontCamera = true;
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
