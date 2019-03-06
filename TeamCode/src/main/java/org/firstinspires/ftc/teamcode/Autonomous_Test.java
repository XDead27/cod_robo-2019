package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.res.AssetManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;

@Autonomous(name = "Autonomous_Test", group = "Autonomous")

public final class Autonomous_Test extends Autonomous_Mode {

    @Override
    protected void runOperations(){

        tfod.shutdown();

        while(opModeIsActive()) {
            if (gamepad1.x) {
                TestWalkAtAngle3();
            }
            else if (gamepad1.y) {
                TestWalkAtAngle4();
            }
            else if (gamepad1.dpad_up) {
                TestEncoderAngle1();
            }
            else if (gamepad1.dpad_down) {
                TestEncoderAngle2();
            }
            else if (gamepad1.dpad_right) {
                TestEncoderAngle3();
            }
            else if(gamepad1.left_bumper){
                Rotate(135);
            }
            else if(gamepad1.right_bumper){
                TestGyro();
            }
            else if(gamepad1.right_trigger > 0.1){
                TestPath();
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

    private void TestWalkAtAngle1(){
        WalkAtAngle(0.5, 30);

        sleep(1000);

        WalkAtAngle(-0.5, 30);

        sleep(1000);

        StopMotors();
    }
    private void TestWalkAtAngle2(){
        WalkAtAngle(0.5, 60);

        sleep(1000);

        WalkAtAngle(-0.5, 60);

        sleep(1000);

        StopMotors();
    }
    private void TestWalkAtAngle3(){
        WalkAtAngle(0.5, -30);

        sleep(1000);

        WalkAtAngle(-0.5, -30);

        sleep(1000);

        StopMotors();
    }
    private void TestWalkAtAngle4(){
        WalkAtAngle(0.5, -60);

        sleep(1000);

        WalkAtAngle(-0.5, -60);

        sleep(1000);

        StopMotors();
    }

    private void TestGyro(){
        while(opModeIsActive() && !gamepad1.dpad_down) {
            telemetry.addData("integrated Gyro", GetAngle());
            telemetry.update();
        }
    }

    private void TestEncoderAngle1(){
        WalkEncoder(10*TICKS_PER_CM, 0.3, 45);
        sleep(1000);
        WalkEncoder(-10*TICKS_PER_CM, 0.5, 45);
        sleep(1000);
    }
    private void TestEncoderAngle2(){
        WalkEncoder(10*TICKS_PER_CM, 0.5, 60);
        sleep(1000);
        WalkEncoder(-10*TICKS_PER_CM, 0.5, 60);
        sleep(1000);
    }
    private void TestEncoderAngle3(){
        WalkEncoder(10*TICKS_PER_CM, 0.5, -60);
        sleep(1000);
        WalkEncoder(-10*TICKS_PER_CM, 0.5, -60);
        sleep(1000);
    }

    private void TestPath(){
        try {

            Activity a = new Activity();
            File file;

            file = new File(a.getAssets().list("")[0]);

            //catch (NullPointerException e){
            //    telemetry.addData("Nu e nimic in folder", " ");
            //    telemetry.update();
            //}

            RunWithPath(file, 0.5);
        } catch(Exception e){
            throw new RuntimeException(e);
        }
    }

    @Override
    protected void endOperations() {


    }

}
