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
    protected void initialise(boolean bIsDriver) {
        super.initialise(bIsDriver);

        CalibrateGyro();
    }

    @Override
    protected void runOperations(){

        tfod.shutdown();

        while(opModeIsActive()) {
            if (gamepad1.a){
                while (opModeIsActive() && !gamepad1.back){
                    telemetry.addData("RangeL address" , RangeL.getI2cAddress());
                    telemetry.addData("RangeR address" , RangeR.getI2cAddress());
                    telemetry.addData("RangeL distance" , RangeL.getDistance(DistanceUnit.CM));
                    telemetry.addData("RangeR distance" , RangeR.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }
                //TestWalkEncoder(argument , 0.5 , 90+45);
            }
            else if(gamepad1.b){
                WalkObstacleAndRangeNORMAL(10, false, 0.9);
            }
            else if (gamepad1.x) {
                MoveToUnlatch();
                ChooseCube(MineralPosition.RIGHT);
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
            else if((gamepad1.left_stick_x > 0.1 || gamepad1.left_stick_y > 0.1) && gamepad1.right_stick_button){
                TestWalkEncoder(argument, 0.5, Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x) + 90);
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


}
