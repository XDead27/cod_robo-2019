package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.res.AssetManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

@Autonomous(name = "Autonomous_Test", group = "Autonomous")
@Disabled


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
                ChooseCube(RIGHT);
            }
            else if (gamepad1.y) {
                //see where the cube is
                LiftPhoneUp();
                MineralPosition now = LEFT;
                LiftPhoneDown();

                MoveSlidersEncoder(1000, 0.5);

                //choose cube
                ChooseCube(now);

                //moves slightly further from the other minerals then turns towards the crater/square
                switch(now){
                    case LEFT:
                        WalkEncoder(20, 0.5, 90);
                        Rotate(-15);
                        break;

                    case MIDDLE:
                        WalkEncoder(5, 0.5, 0);
                        break;

                    case RIGHT:
                        WalkEncoder(20, 0.5, -90);
                        Rotate(15);
                        break;
                }

                //park
                ParkAtCrater();
            }

            else if (gamepad1.dpad_up) {
                //MIDDLE DANI, MIDDLE
                LiftDown();

                //move left
                MoveToUnlatch();

                //see where the cube is
                LiftPhoneUp();
                MineralPosition now = MIDDLE;
                LiftPhoneDown();

                MoveSlidersEncoder(1000, 0.5);

                //choose cube
                ChooseCube(now);

                WalkEncoder(5, 0.5, 0);

                //park
                ParkAtCrater();

            }
            else if (gamepad1.dpad_down) {

            }
            else if (gamepad1.dpad_left) {
                //LEFT AN POOLA MEA
                LiftDown();

                //move left
                MoveToUnlatch();

                //see where the cube is
                LiftPhoneUp();
                MineralPosition now = LEFT;
                LiftPhoneDown();

                MoveSlidersEncoder(1000, 0.5);

                //choose cube
                ChooseCube(now);

                //moves slightly further from the other minerals then turns towards the crater/square

                WalkEncoder(20, 0.5, 90);
                Rotate(-15);

                //park
                ParkAtCrater();
            }
            else if (gamepad1.dpad_right) {
                //RIGHT
                LiftDown();

                //move left
                MoveToUnlatch();

                //see where the cube is
                LiftPhoneUp();
                MineralPosition now = RIGHT;
                LiftPhoneDown();

                MoveSlidersEncoder(1000, 0.5);

                //choose cube
                ChooseCube(now);

                //moves slightly further from the other minerals then turns towards the crater/square
                WalkEncoder(20, 0.5, -90);
                Rotate(15);

                //park
                ParkAtCrater();
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
