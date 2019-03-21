package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Set;

import static org.firstinspires.ftc.teamcode.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.MineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.MineralPosition.RIGHT;

public abstract class Autonomous_Mode extends RobotHardwareClass {

    protected static int TICKS_PER_CM = 15; //TODO: chiar trebuie sa il aflam
    protected static int DIST_GLISIERE = 2500;
    protected static int EXTINDERE_MAX = 3500;
    protected static double TOLERANCE = 0.0001;
    protected static double DIAGONAL_CONSTANT = 2.0;
    protected static double VERTICAL_CONSTANT = 1.0;
    protected static double HORIZONTAL_CONSTANT = 1.5;
    //Integrated gyro variables
    Orientation lastAngles = new Orientation();
    double globalAngle;

    //functii abstrcte
    protected abstract void runOperations();

    protected abstract void endOperations();

    @Override
    public void runOpMode() throws  InterruptedException{
        initialise(false);

        waitForStart();

        runOperations();

        endOperations();
    }

    //************
    //VUFORIA
    //************


    //We use the phone to identify the cube's position by looking at the 2 minerals in the left and using an A.I. that recognises their type
    //If we see a cube, we return its position
    //If we have 2 spheres, that means that the cube is on the right
    //The function returns an enum that is eithr LEFT, MIDDLE or RIGHT, coresponding to the cube's position

    protected MineralPosition Position(int elem) {

        //TODO : sa inteleg ce este top, bottom, left, right (daca sunt fata de mardinile ecranului sau altceva), dam telemetry sa aflam
        //TODO : atunci cand sunt la crater sa incerc sa nu iau din greseala elem din spate

        MineralPosition ret = RIGHT;
        tfod.activate();

        if (elem == 2) {
            int MaxTime = 4000;
            int CurTime = 0;
            int period = 5;
            while (opModeIsActive() && CurTime < MaxTime) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null && updatedRecognitions.size() >= 2) {
                    String label1 = "";
                    String label2 = "";
                    int top1 = -1; //top de la tel cand e vertical , deci left pt landscape
                    int top2 = -1;
                    int left1 = 100000; //top de la tel cand e vertical , deci buttom pt landscape
                    int left2 = 100000;
                    for (Recognition recognition : updatedRecognitions) {
                        if (left1 > (int) recognition.getLeft()) {
                            left2 = left1;
                            top2 = top1;
                            label2 = label1;

                            left1 = (int) recognition.getLeft();
                            top1 = (int) recognition.getTop();
                            label1 = recognition.getLabel();
                        } else if (left2 > (int) recognition.getLeft()) {
                            left2 = (int) recognition.getLeft();
                            top2 = (int) recognition.getTop();
                            label2 = recognition.getLabel();
                        }
                    }
                    if (top1 != -1 && top2 != -1) {
                        if (label1.equals(LABEL_GOLD_MINERAL) || label2.equals(LABEL_GOLD_MINERAL)) {
                            if (label1.equals(LABEL_GOLD_MINERAL)) {
                                if (top1 > top2) {
                                    ret = LEFT;
                                } else {
                                    ret = MIDDLE;
                                }
                            } else {
                                if (top2 > top1) {
                                    ret = LEFT;
                                } else {
                                    ret = MIDDLE;
                                }
                            }
                        } else {
                            ret = RIGHT;
                        }
                        break;
                    }
                }
                sleep(period);
                CurTime+=period;
            }
        } else if (elem == 3) {
            while (opModeIsActive()) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null && updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getTop();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getTop();
                        } else {
                            silverMineral2X = (int) recognition.getTop();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            return LEFT;
                        } else if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            return RIGHT;
                        } else {
                            return MIDDLE;
                        }
                    }
                }
            }
        }

        tfod.shutdown();
        return ret;
    }

    //************
    //MISCARE
    //************

    //Set the motors power individually
    protected void SetWheelsPower(double FL, double FR, double BL, double BR) {
        MotorFR.setPower(FR);
        MotorBL.setPower(BL);
        MotorFL.setPower(FL);
        MotorBR.setPower(BR);
    }

    //Overcharge of the previous function, takes only two arguments and sets the wheels power based
    //on mecanum wheel dependency
    protected void SetWheelsPower(double FLBR, double FRBL) {
        MotorFR.setPower(FRBL);
        MotorBL.setPower(FRBL);
        MotorFL.setPower(FLBR);
        MotorBR.setPower(FLBR);
    }

    //Provided with the speed and the angle in degrees(relative to the current rotation), make the
    //robot pan-move in that direction
    protected void WalkAtAngle(double speed, double angle) {

        if (speed < 0) {
            speed *= -1;
            angle += 180;
        }

        //to translate from the trigonometric form to oriented XoY axing(just like on the controllers)
        angle = angle + 90;

        //Transform from angle to vectorial distribution(0 is 0%, 1 is 100%, you could think of it
        //as the proportion of X and Y)
        double AxisXVector = Math.cos(Math.toRadians(angle));
        double AxisYVector = Math.sin(Math.toRadians(angle));

        //Power the wheel motors according to the vectorial distribution
        //The function that describes the vectorial distribution is
        double VectorFLBR = Range.clip(MecanumFunctionCalculator(AxisXVector, AxisYVector, true) , -1 , 1);
        double VectorFRBL = Range.clip(MecanumFunctionCalculator(AxisXVector, AxisYVector, false) , -1 , 1);

        //Scale that vector by the desired speed, keeping in mind the maximum
        speed = Range.clip(speed, 0, 1);
        double ScalingCoefficient = speed / Math.max(Math.abs(VectorFLBR), Math.abs(VectorFRBL));

        double SpeedFLBR = VectorFLBR * ScalingCoefficient;
        double SpeedFRBL = VectorFRBL * ScalingCoefficient;

        //Set the power of the wheels
        SetWheelsPower(SpeedFLBR, SpeedFRBL);
    }

    //Pan-move a desired distance, being given the speed and the angle as well
    protected void WalkEncoder(double dist, double speed, double angle) {

        //TODO : merge mult mai mult ca dist (ii dadeam 10 cm, el mergea un metru, s-ar putea sa fie o greseala de la calcule sau encodere, dam telemetry sa aflam

        ResetAllEncoders();

        RunWithAllEncoders();

        dist *= TICKS_PER_CM;

        if (angle % 180 == 0){
            dist *= VERTICAL_CONSTANT;
        }
        else if (angle % 90 == 0){
            dist *= HORIZONTAL_CONSTANT;
        }
        else {
            dist *= DIAGONAL_CONSTANT;
        }

        boolean bSpecial = false;
        if (angle % 90 == 0){
            bSpecial = true;
        }

        if (dist < 0) {
            dist *= -1;
            angle += 180;
        }

        WalkAtAngle(speed, angle);

        //to translate from the trigonometric form to oriented XoY axing(just like on the controllers)
        angle += 90;

        //calculate vectors based on the given angle and the distance
        double TargetXVector = Math.cos(Math.toRadians(angle));
        double TargetYVector = Math.sin(Math.toRadians(angle));

        ///calculate the encoder target
        //use the mecanum function calculator just like you would for normal speeds, but now it will
        //return the distance that each wheel will travel to reach the target distance with the target angle
        double TargetFLBR = dist * MecanumFunctionCalculator(TargetXVector, TargetYVector, true);
        double TargetFRBL = dist * MecanumFunctionCalculator(TargetXVector, TargetYVector, false);

        double FLBRMean = 0;
        double FRBLMean = 0;

        ///making run to position by hand, because the normal function sometimes has bugs
        //the motors will not stop unless they have overpassed their target distance
        while ((Math.abs(TargetFRBL) > Math.abs(FRBLMean) || Math.abs(TargetFLBR) > Math.abs(FLBRMean)) && opModeIsActive()) {

            FLBRMean = (double)(Math.abs(MotorFL.getCurrentPosition()) + Math.abs(MotorBR.getCurrentPosition())) / 2.0;
            FRBLMean = (double)(Math.abs(MotorFR.getCurrentPosition()) + Math.abs(MotorBL.getCurrentPosition())) / 2.0;
            if (bSpecial){
                FLBRMean = (FLBRMean + FRBLMean) / 2.0;
                FRBLMean = FLBRMean;
            }

            //telemetry.addData("angle" , angle);
            //telemetry.addData("TargetXVector" , TargetXVector);
            //telemetry.addData("TargetYVector" , TargetYVector);

            telemetry.addData("TargetFLBR" , TargetFLBR);
            telemetry.addData("FLBRMean" , FLBRMean);
            telemetry.addData("TargetFRBL" , TargetFRBL);
            telemetry.addData("FRBLMean" , FRBLMean);

            telemetry.addData("FL" , MotorFL.getCurrentPosition());
            telemetry.addData("FR" , MotorFR.getCurrentPosition());
            telemetry.addData("BL" , MotorBL.getCurrentPosition());
            telemetry.addData("BR" , MotorBR.getCurrentPosition());

            telemetry.update();

            idle();
        }

        //stop motors and end function
        StopMotors();
    }

    //Complex moving function which avoids incoming obstacles and aligns with the wall at the end. It
    //uses a double PID system with two range sensors, each one of them for a side of the robot
    protected void WalkObstacleAndRangeNORMAL(double distanceFromWall, boolean bStartAlignedWithWall, double PowerMax) {
        //IN-DEPTH EXPLANATION
        //
        //  This is a function that makes the robot move forward until there is a target distance
        //between it and the wall in front of him (distanceFromWall) for a certain amount of time.
        //  It has the option to align the robot with the wall in front of him before starting to move
        //(bStartAlignedWithWall).
        //  There is a timer loop that acts like a timer, constantly checking the sensors & checking if
        //the target distance from wall is reached and maintained for a number of seconds (delay). The
        //loop operates at a given rate (period) to add to the timer every iteration.
        //  If the robot meets an obstacle there are two possible outcomes: if the obstacle is detected
        //by only one sensor (it doesn't come perfectly head on) then the robot will execute a quick
        //one wheel axis turn; if both the sensor report an obstacle (the obstacle comes head on)
        //then the robot will simply move backwards.
        //  If the robot doesn't move from target distance for [delay] milliseconds then the function
        //is exited.


        //if wanted, the robot aligns with the wall before starting to move towards it
        if (bStartAlignedWithWall) {
            while (Math.abs(RangeL.rawUltrasonic() - RangeL.rawUltrasonic()) < 10/*magic number*/) {
                int dir = (int) Math.signum(RangeL.rawUltrasonic() - RangeL.rawUltrasonic());
                SetWheelsPower(-0.2 * dir, 0.2 * dir, -0.2 * dir, 0.2 * dir);
            }
        }

        //initialising of the timer loop
        final double target = distanceFromWall;
        final double delay = 2000;
        final long period = 10L; //while ce opereaza la frecventa de 10 ms

        boolean bIsUsingEncoder = false;

        ///IMPLEMENTATION GYRO
        ResetAngle();
        double currentHeading;

        double pGain = PowerMax / (target - 5); //daca zidul sau alt robot se apropie mai mult decat trebuie atunci sa mearga la viteza maxima in spate
        double dGain = 0.0;

        double errorRight = target - RangeL.getDistance(DistanceUnit.CM);
        double errorLeft = target - RangeL.getDistance(DistanceUnit.CM);

        double proportionalSpeedLeft = 0;
        double proportionalSpeedRight = 0;

        double finalSpeedLeft = 0, finalSpeedRight = 0, lastSpeedLeft = finalSpeedLeft, lastSpeedRight = finalSpeedRight;

        double initValueL = 0, initValueR = 0;

        float steadyTimer = 0;

        //Implement acceleration so that the big changes in sensors input do not drastically affect
        //the robot's speed
        double Acceleration = 0.2;
        final double INIT_ACCELERATION = 0.2, MAX_ACCELERATION = 1.0, ACCELERATION_INCREMENT = 0.3;

        while (opModeIsActive() && steadyTimer < delay) {

            //*****************************
            //timer condition
            if (Math.abs(errorLeft) < 2 || Math.abs(errorRight) < 2) {
                steadyTimer += period;
            } else {
                steadyTimer = 0;
            }


            //****************************
            //PID
            errorRight = target - RangeL.getDistance(DistanceUnit.CM);
            errorLeft = target - RangeL.getDistance(DistanceUnit.CM);

            proportionalSpeedRight = (errorRight * pGain);
            proportionalSpeedLeft = (errorLeft * pGain);

            //the outcome will be negative (because of the distance subtraction) so we invert it
            finalSpeedLeft = -proportionalSpeedLeft;
            finalSpeedRight = -proportionalSpeedRight;

            //we restrict the outcome to the minimum and maximum for the motor
            finalSpeedLeft = Range.clip(finalSpeedLeft, -0.9, 0.7);
            finalSpeedRight = Range.clip(finalSpeedRight, -0.9, 0.7);

            //****************************
            //exceptions (if the output is a really small number)
            if (Math.abs(finalSpeedLeft) < TOLERANCE) {
                finalSpeedLeft = 0;
            }
            if (Math.abs(finalSpeedRight) < TOLERANCE) {
                finalSpeedRight = 0;
            }

            //if the robot has to avoid an obstacle
            if (finalSpeedLeft > 0 && finalSpeedRight < 0) {
                finalSpeedLeft = 0;
            }
            if (finalSpeedRight > 0 && finalSpeedLeft < 0) {
                finalSpeedRight = 0;
            }

            //****************************
            //gyro
            currentHeading = GetAngle();
            if (currentHeading > 180) {
                currentHeading = currentHeading - 360;
            }
            if (Math.abs(currentHeading) > 2 && !bIsUsingEncoder) {
                bIsUsingEncoder = true;
                initValueL = MotorFL.getCurrentPosition();
                initValueR = MotorFR.getCurrentPosition();
            }

            //****************************
            //get back on previous track (once the obstacle has passed)
            if (bIsUsingEncoder) {
                if (initValueL <= MotorFL.getCurrentPosition()) {
                    finalSpeedLeft = 0;
                }
                if (initValueR <= MotorFR.getCurrentPosition()) {
                    finalSpeedRight = 0;
                }

                if (initValueL < MotorFL.getCurrentPosition() && initValueR < MotorFR.getCurrentPosition()) {
                    bIsUsingEncoder = false;
                    ResetAngle();
                }
            }

            //Acceleration rules
            //If the sensors tell the robot to move in the oposite direction or if the final speed is zero then reset the acceleration
            if(Math.signum(lastSpeedLeft) == Math.signum(finalSpeedLeft) && Math.signum(lastSpeedRight) == Math.signum(finalSpeedRight) && finalSpeedLeft != 0 && finalSpeedLeft != 0){
                Acceleration += (period / 1000) * ACCELERATION_INCREMENT;
                Acceleration = Range.clip(Acceleration, INIT_ACCELERATION, MAX_ACCELERATION);
            }
            else{
                Acceleration = INIT_ACCELERATION;
            }

            SetWheelsPower(finalSpeedLeft * Acceleration, finalSpeedRight * Acceleration, finalSpeedLeft * Acceleration, finalSpeedRight * Acceleration);

            telemetry.addData("using encoder ", bIsUsingEncoder);
            telemetry.addData("speed left", finalSpeedLeft);
            telemetry.addData("speed right", finalSpeedRight);
            telemetry.addData("error left ", errorLeft);
            telemetry.addData("error right ", errorRight);
            telemetry.addData("steady timer ", steadyTimer);
            telemetry.update();

            lastSpeedLeft = finalSpeedLeft;
            lastSpeedRight = finalSpeedRight;

            sleep(period);
        }
        StopMotors();
    }

    //TODO
    protected void WalkObstacleAndRangeSTRAFE(double distanceFromWall, boolean bStartAllignedWithWall) {
        //se aliniaza cu zidul din fata
        if (bStartAllignedWithWall) {
            while (Math.abs(RangeL.rawUltrasonic() - RangeL.rawUltrasonic()) < 10/*magic number*/) {
                int dir = (int) Math.signum(RangeL.rawUltrasonic() - RangeL.rawUltrasonic());
                SetWheelsPower(-0.2 * dir, 0.2 * dir, -0.2 * dir, 0.2 * dir);
            }
        }

        final double target = distanceFromWall;
        final double delay = 2000;
        final long period = 10L; //while ce opereaza la frecventa de 10 ms

        boolean bIsUsingEncoder = false, bHasFinishedAvoiding = false;

        double pGain = 1 / (target - 5); //daca zidul sau alt robot se apropie mai mult decat trebuie atunci sa mearga la viteza maxima in spate
        double dGain = 0.0;

        double errorRight = target - RangeL.getDistance(DistanceUnit.CM);

        double errorLeft = target - RangeL.getDistance(DistanceUnit.CM);

        double proportionalSpeedLeft = 0;
        double proportionalSpeedRight = 0;

        double finalSpeedLeft = 0, finalSpeedRight = 0;

        double initValueFL = 0, initValueFR = 0;

        float steadyTimer = 0;

        while (opModeIsActive() && steadyTimer < delay) {

            //*****************************
            //conditia de timer
            if (Math.abs(errorLeft) < 2 || Math.abs(errorRight) < 2) {
                steadyTimer += period;
            } else {
                steadyTimer = 0;
            }


            //****************************
            //PID
            errorRight = target - RangeL.getDistance(DistanceUnit.CM);
            errorLeft = target - RangeL.getDistance(DistanceUnit.CM);

            proportionalSpeedRight = (errorRight * pGain);
            proportionalSpeedLeft = (errorLeft * pGain);

            //inversam viteza ca sa fie pozitiva
            finalSpeedLeft = -proportionalSpeedLeft;
            finalSpeedRight = -proportionalSpeedRight;

            finalSpeedLeft = Range.clip(finalSpeedLeft, -0.9, 0.7);
            finalSpeedRight = Range.clip(finalSpeedRight, -0.9, 0.7);

            //****************************
            //exceptii
            if (Math.abs(finalSpeedLeft) < TOLERANCE) {
                finalSpeedLeft = 0;
            }
            if (Math.abs(finalSpeedRight) < TOLERANCE) {
                finalSpeedRight = 0;
            }

            //if the robot has to avoid
            if (finalSpeedLeft > 0 && finalSpeedRight < 0) {
                WalkAtAngle(finalSpeedLeft, 90);
                if (!bIsUsingEncoder) {
                    initValueFL = MotorFL.getCurrentPosition();
                    initValueFR = MotorFR.getCurrentPosition();
                }
                bIsUsingEncoder = true;
            } else if (finalSpeedRight > 0 && finalSpeedLeft < 0) {
                WalkAtAngle(finalSpeedLeft, 90);
                if (!bIsUsingEncoder) {
                    initValueFL = MotorFL.getCurrentPosition();
                    initValueFR = MotorFR.getCurrentPosition();
                }
                bIsUsingEncoder = true;
            } else {
                StopMotors();
                bHasFinishedAvoiding = true;
            }


            //****************************
            //retrack
            if (bHasFinishedAvoiding && bIsUsingEncoder) {
                //TODO
            }

            SetWheelsPower(finalSpeedLeft, finalSpeedRight, finalSpeedLeft, finalSpeedRight);

            telemetry.addData("using encoder ", bIsUsingEncoder);
            telemetry.addData("speed left", finalSpeedLeft);
            telemetry.addData("speed right", finalSpeedRight);
            telemetry.addData("error left ", errorLeft);
            telemetry.addData("error right ", errorRight);
            telemetry.addData("steady timer ", steadyTimer);
            telemetry.update();

            sleep(period);
        }
        StopMotors();
    }

    //stop all motors
    protected void StopMotors() {
        MotorBL.setPower(0);
        MotorBR.setPower(0);
        MotorFL.setPower(0);
        MotorFR.setPower(0);
    }

    //ma rotesc la angle grade; negativ e pentru right, pozitiv e pentru left;
    protected void Rotate(double angle) {

        double FinalAngle = GetAngle() + angle;

        double FLPower = -0.4;
        double BLPower = -0.4;
        double FRPower = 0.4;
        double BRPower = 0.4;

        if (angle < 0) {
            FLPower *= -1;
            BLPower *= -1;
            FRPower *= -1;
            BRPower *= -1;
        }

        SetWheelsPower(FLPower, FRPower, BLPower, BRPower);

        while (opModeIsActive() && Math.abs(FinalAngle - GetAngle()) > 15) {
            telemetry.addData("sunt in modul normal", FinalAngle);
            telemetry.addData(" m-am rotit pana la ", GetAngle());
            telemetry.update();
            idle();
        }

        SetWheelsPower(FLPower/2, FRPower/2, BLPower/2, BRPower/2);


        while (opModeIsActive() && Math.abs(FinalAngle - GetAngle()) > 5) {
            telemetry.addData("sunt in modul incet", " ");
            telemetry.addData("m-am rotit pana la ", GetAngle());
            telemetry.update();
            idle();
        }

        StopMotors();
    }

    //align with wall
    protected void AlignWithWall() {
        if (RangeL.getDistance(DistanceUnit.CM) > RangeR.getDistance(DistanceUnit.CM)){

            SetWheelsPower(0.3, -0.3, 0.3, -0.3);

            while (opModeIsActive() && RangeL.getDistance(DistanceUnit.CM) - RangeR.getDistance(DistanceUnit.CM) > 0) {
                idle();
            }
        }
        else{
            SetWheelsPower(-0.3, 0.3, -0.3, 0.3);

            while (opModeIsActive() && RangeR.getDistance(DistanceUnit.CM) - RangeL.getDistance(DistanceUnit.CM) > 0) {
                idle();
            }
        }


        StopMotors();
    }

    //Follow a Text Editor pah
    protected void RunWithPath(File inFile, double OverallSpeed) throws Exception {
        File file = inFile;
        CourseReaderClass Crc = new CourseReaderClass(file);

        List<Double> AngleList = Crc.GetAngleMap();
        List<Double> VectorList = Crc.GetVectorMap();

        for (int i = 0; i < Math.min(AngleList.size(), VectorList.size()); i++) {
            WalkEncoder(VectorList.get(i) * 67, OverallSpeed, AngleList.get(i));
        }
    }


    //************
    //GYRO REV
    //************

    protected void CalibrateGyro(){
        BNO055IMU.Parameters REVGyroParameters = new BNO055IMU.Parameters();

        REVGyroParameters.mode = BNO055IMU.SensorMode.IMU;
        REVGyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        REVGyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        REVGyroParameters.loggingEnabled = false;


        imuGyro.initialize(REVGyroParameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imuGyro.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imuGyro.getCalibrationStatus().toString());
        telemetry.update();
    }

    //Function that adds the orientation of the REV integrated gyro to the globalAngle variable
    protected double GetAngle() {
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imuGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    //Function that resets the global angle to 0
    protected void ResetAngle() {
        lastAngles = imuGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }


    //************
    //MISCELLANEOUS
    //************

    //Function that returns either FLBR or FRBL motors speeds based on a vectorial distribution
    protected double MecanumFunctionCalculator(double VectorX, double VectorY, boolean bFLBR) {
        //IN-DEPTH EXPLANATION
        //
        //  This function would normally be two separate functions.
        //  We found out, quite experimentally, that this is the function that perfectly describes
        //the values that each motor power should have depending on the given vector values
        //  The simple function that approximately describes the desired behavior would be y - x
        //(for FLBR) and y + x (for FRBL). We tried squaring each variable (because of Pythagoras
        //theorem - we wanted to find out the hypotenuse of a triangle with the sizes x and y),
        //but this turned out to cancel the sign of each parameter y^2 - x^2, x,y > 0 is equal to
        //y^2 - x^2, x,y < 0. So the obvious solution was to multiply the squared parameter with the
        //sign of each variable.
        //  This is how we ended up with the function f(x, y) = sig(y)*(y^2) - sig(x)*(x^2) and its
        //conjugate

        //return bFLBR ? (Math.signum(VectorY) * Math.pow(VectorY, 2)) + (Math.signum(VectorX) * Math.pow(VectorX, 2)) : (Math.signum(VectorY) * Math.pow(VectorY, 2)) - (Math.signum(VectorX) * Math.pow(VectorX, 2));
        //return bFLBR? VectorY + VectorX : VectorY - VectorX;
        return Range.clip(bFLBR? VectorY + VectorX : VectorY - VectorX, -1, 1);
    }

    //Reset all encoders
    protected void ResetAllEncoders() {
        MotorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void RunWithAllEncoders() {
        MotorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void LiftDown() {

        MotorGlisieraL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGlisieraR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGlisieraL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorGlisieraR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorGlisieraL.setPower(-0.3);
        MotorGlisieraL.setPower(-0.3);

        sleep(500);

        MotorGlisieraL.setPower(0.3);
        MotorGlisieraR.setPower(0.3);
        SetWheelsPower(0 , 0 , -0.3 , -0.3);

        while (Math.abs(MotorGlisieraR.getCurrentPosition()) < DIST_GLISIERE && opModeIsActive()) {
            telemetry.addData("encoder" , MotorGlisieraR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        StopMotors();

        MotorGlisieraL.setPower(0.7);
        MotorGlisieraR.setPower(0.7);
        sleep(100);

        StopGlisiere();

        CalibrateGyro();
    }

    protected void MoveSlidersEncoder(int dist , double speed) {

        //dist < 0 -> glisierele coboara
        //dist > 0 -> glisierele urca

        speed = Math.abs(speed);

        int dir = 1;

        if (dist < MotorGlisieraR.getCurrentPosition()){
            speed *= -1;
            dir = -1;
        }

        MotorGlisieraL.setPower(speed);
        MotorGlisieraR.setPower(speed);

        while (MotorGlisieraR.getCurrentPosition()*dir < dist*dir && opModeIsActive()) {
            telemetry.addData("glisiere" , MotorGlisieraR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        StopGlisiere();
    }

    protected void LiftPhoneUp() {
        ServoPhone.setPosition(0.115);
    }

    protected void LiftPhoneDown() {
        ServoPhone.setPosition(0.5);
    }

    protected void ExtendSlidingSystem() {
        MotorExtindere.setPower(0.7);
        while (Math.abs(MotorExtindere.getCurrentPosition()) < EXTINDERE_MAX && opModeIsActive()){
            telemetry.addData("extindere" , MotorExtindere.getCurrentPosition());
            telemetry.update();
            idle();
        }
        MotorExtindere.setPower(0);
    }

    protected void GetObjects() {
        while (opModeIsActive()){
            MotorRotirePerii.setPower(-0.7);
        }
    }

    protected void PlantTeamMarker() {
        MotorRotirePerii.setPower(0.7);
        sleep(2000);
        MotorRotirePerii.setPower(0);
    }

    protected void StopGlisiere(){
        MotorGlisieraL.setPower(0);
        MotorGlisieraR.setPower(0);
    }

    protected void LiftSlidersUpABit(){
        MoveSlidersEncoder(500 , 0.5);
    }

    protected void MoveToUnlatch(){
        WalkEncoder(8 , 0.5 , 90);
    }

    protected void ChooseCube(MineralPosition now){
        if (now == LEFT){
            WalkEncoder(20 , 0.5 , 0);
            WalkEncoder(30 , 0.5 , 45);
            WalkEncoder(10 , 0.5 , 0);
        }
        else if (now == MIDDLE){
            WalkEncoder(20 , 0.5 , 0);
            WalkEncoder(30 , 0.5 , -45);
            WalkEncoder(10 , 0.5 , 0);
        }
        else if (now == RIGHT){
            WalkEncoder(35 , 0.5 , -45);
            WalkEncoder(20 , 0.5 , -90);
            WalkEncoder(10 , 0.5 , 0);
        }
    }

    protected void LetTeamMarker(MineralPosition now){
        if (now == LEFT){
            WalkEncoder(30 , 0.5 , 0);
            Rotate(-135);
            WalkEncoder(70 , 0.5 , 90);
        }
        else if (now == MIDDLE){
            WalkEncoder(55 , 0.5 , 0);
            Rotate(-100);
        }
        else if (now == RIGHT){
            WalkEncoder(45 , 0.5 , 0);
            Rotate(-70);
            WalkEncoder( 30, 0.5, 90);
        }

        PlantTeamMarker();
    }

    protected void ContinuareCrater(MineralPosition now){
        if(now == LEFT){
            Rotate(-20);
        }
        else if(now == RIGHT){
            Rotate(10);
        }
    }

    protected void ParkAtCrater(){
        //lower the sliders
        MoveSlidersEncoder(200 , 0.5);

        //extend the sliders
        ExtendSlidingSystem();

        //try to capture objects until the end
        GetObjects();
    }

    protected void GoBackAndTurn(){
        WalkEncoder(-10 , 0.5 , 0);
        Rotate(90);
    }

    protected void WalkToWall(){
        double DeadZoneRange = 2;
        SetWheelsPower(0.3 , 0.3);
        while (!(RangeL.getDistance(DistanceUnit.CM) > DeadZoneRange && RangeL.getDistance(DistanceUnit.CM) < 20) && !(RangeR.getDistance(DistanceUnit.CM) > DeadZoneRange && RangeR.getDistance(DistanceUnit.CM) < 20)){
            idle();
        }
        StopMotors();
    }

}