package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Test", group = "Autonomous")

public final class Autonomous_Test extends Autonomous_Mode {

    @Override
    protected void runOperations() {

        tfod.shutdown();

        while(opModeIsActive()) {
            if (gamepad1.a) {
                TestWalkAtAngle1();
            }
            else if (gamepad1.b) {
                TestWalkAtAngle2();
            }
            else if (gamepad1.x) {
                TestWalkAtAngle3();
            }
            else if (gamepad1.y) {
                TestWalkAtAngle4();
            }
            else if (gamepad1.dpad_up) {
                TestEncoderAngle();
            }
            else{
                idle();
            }
        }

    }

    void TestPosition(){
        MineralPosition now = Position();
        while(opModeIsActive() && !gamepad1.dpad_down){
            telemetry.addData("position : " , now);
            telemetry.update();
            idle();
        }
    }

    void TestWalkAtAngle1(){
        WalkAtAngle(0.5, 30);

        sleep(1000);

        WalkAtAngle(-0.5, 30);

        sleep(1000);

        StopMotors();
    }
    void TestWalkAtAngle2(){
        WalkAtAngle(0.5, 60);

        sleep(1000);

        WalkAtAngle(-0.5, 60);

        sleep(1000);

        StopMotors();
    }
    void TestWalkAtAngle3(){
        WalkAtAngle(0.5, -30);

        sleep(1000);

        WalkAtAngle(-0.5, -30);

        sleep(1000);

        StopMotors();
    }
    void TestWalkAtAngle4(){
        WalkAtAngle(0.5, -60);

        sleep(1000);

        WalkAtAngle(-0.5, -60);

        sleep(1000);

        StopMotors();
    }

    void TestGyro(){
        while(opModeIsActive() && !gamepad1.dpad_down) {
            telemetry.addData("integrated Gyro", GetAngle());
            telemetry.update();
        }
    }

    void TestEncoderAngle(){
        WalkEncoder(10*67, 0.5, 45);
        WalkEncoder(-10*67, 0.5, 45);
    }

    @Override
    protected void endOperations() {


    }

}
