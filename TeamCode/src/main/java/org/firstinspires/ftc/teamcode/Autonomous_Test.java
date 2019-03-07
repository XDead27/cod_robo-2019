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

        //tfod.shutdown();

        while(opModeIsActive()) {
            if (gamepad1.a){
                TestPosition(2);
                return;
            }
            else if(gamepad1.b){
                TestPosition(3);
                return;
            }
            else if (gamepad1.x) {
                TestWalkAtAngle(0.5 , 45);
            }
            else if (gamepad1.y) {

            }
            else if (gamepad1.dpad_up) {

            }
            else if (gamepad1.dpad_down) {

            }
            else if (gamepad1.dpad_left) {
                TestWalkEncoder(10 , 0.5 , 0);
            }
            else if (gamepad1.dpad_right) {

            }
            else if(gamepad1.left_bumper){
                Rotate(135);
            }
            else if(gamepad1.right_bumper){
                TestGyro();
            }
            else if(gamepad1.left_trigger > 0.1){
                TestPath();
            }
            else if(gamepad1.right_trigger > 0.1){

            }
            else{
                idle();
            }

        }

    }

    @Override
    protected void endOperations() {


    }

    void TestPosition(int elem){
        MineralPosition now = Position(elem);
        while(opModeIsActive() && !gamepad1.dpad_down){
            telemetry.addData("position : " , now);
            telemetry.update();
            idle();
        }
    }

    private void TestWalkAtAngle(double speed , double angle){
        WalkAtAngle(speed, angle);

        sleep(1000);

        WalkAtAngle(-speed, angle);

        sleep(1000);

        StopMotors();
    }

    private void TestGyro(){
        while(opModeIsActive() && !gamepad1.dpad_down) {
            telemetry.addData("integrated Gyro", GetAngle());
            telemetry.update();
        }
    }

    private void TestWalkEncoder(double dist , double speed , double angle){
        WalkEncoder(dist*TICKS_PER_CM, speed, angle);
        sleep(1000);
        WalkEncoder(-dist*TICKS_PER_CM, speed, angle);
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

}
