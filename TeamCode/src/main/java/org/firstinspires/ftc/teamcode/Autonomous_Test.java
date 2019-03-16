package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.res.AssetManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;

@Autonomous(name = "Autonomous_Test", group = "Autonomous")

public final class Autonomous_Test extends Autonomous_Mode {

    double argument = 0;
    double acceleration = 1.0;
    double delay = 10.0;

    @Override
    protected void runOperations(){

        tfod.shutdown();

        while(opModeIsActive()) {
            if (gamepad1.a){
                while (opModeIsActive() && !gamepad1.b){
                    telemetry.addData("RangeL" , RangeL.getI2cAddress());
                    telemetry.addData("RangeR" , RangeR.getI2cAddress());
                    telemetry.addData("RangeL" , RangeL.getDistance(DistanceUnit.CM));
                    telemetry.addData("RangeR" , RangeR.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }
                //TestWalkEncoder(argument , 0.5 , 90+45);
            }
            else if(gamepad1.b){
                //TestWalkEncoder(argument , 0.5 , 180+45);

            }
            else if (gamepad1.x) {
                //TestWalkEncoder(argument , 0.5 , 45);
                //PlantTeamMarker();
            }
            else if (gamepad1.y) {
                //TestWalkEncoder(argument , 0.5 , -45);
            }
            else if (gamepad1.dpad_up) {
                TestWalkEncoder(argument , 0.5 , 0);
            }
            else if (gamepad1.dpad_down) {
                TestWalkEncoder(argument , 0.5 , 180);
            }
            else if (gamepad1.dpad_left) {
                TestWalkEncoder(argument , 0.5 , 90);
            }
            else if (gamepad1.dpad_right) {
                TestWalkEncoder(argument , 0.5 , -90);
            }
            else if(gamepad1.left_bumper){
                Rotate(argument);
            }
            else if(gamepad1.right_bumper){
                TestGyro();
            }
            else if(gamepad1.left_trigger > 0.1){
                //TestPath();
            }
            else if(gamepad1.right_trigger > 0.1){

            }
            else{
                idle();
            }

            if(Math.abs(gamepad1.right_stick_y) > 0.1){
                argument += -gamepad1.right_stick_y * (delay / 1000) * acceleration;
                acceleration += (delay / 1000) * 4;
            }else{
                acceleration = 1;
            }

            telemetry.addData("Argument :", argument);
            telemetry.update();

            sleep((long)delay);
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
        WalkEncoder(dist, speed, angle);
        //sleep(1000);
        //WalkEncoder(-dist*TICKS_PER_CM, speed, angle);
        //sleep(1000);
    }

    /*private void TestPath(){
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
    }*/

}
