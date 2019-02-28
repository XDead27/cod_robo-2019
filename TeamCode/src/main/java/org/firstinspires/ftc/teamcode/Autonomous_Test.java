package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Test", group = "Autonomous")

public final class Autonomous_Test extends Autonomous_Mode {

    @Override
    protected void runOperations() {

        boolean test_Position = true;
        boolean test_WalkAtAngle = false;

        if (test_Position){
            TestPosition();
        }
        if (test_WalkAtAngle){
            TestWalkAtAngle();
        }

    }

    void TestPosition(){
        MineralPosition now = Position();
        while(opModeIsActive()){
            telemetry.addData("position : " , now);
            telemetry.update();
            idle();
        }
    }

    void TestWalkAtAngle(){
        WalkAtAngle(1, 45);

        while(opModeIsActive()){
            idle();
        }

        StopMotors();
    }

    @Override
    protected void endOperations() {


    }

}
