package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class Autonomous_Mode extends LinearOpMode {

    //motoare roti
    protected DcMotor Motor_FL = null;
    protected DcMotor Motor_FR = null;
    protected DcMotor Motor_BL = null;
    protected DcMotor Motor_BR = null;

    //motoare mecanisme

    //motoare servo
    protected Servo servo_L = null;
    protected Servo servo_R = null;

    //senzori
    protected ModernRoboticsI2cColorSensor color = null;
    protected ModernRoboticsI2cRangeSensor RangeL = null;
    protected ModernRoboticsI2cRangeSensor RangeR = null;
    protected ModernRoboticsI2cGyro gyro = null;

    //constante
    protected final int tics_per_cm = 67;
    protected static double TOLERANCE = 0.0001;

    //functii abstrcte
    protected abstract void runOperations();
    protected abstract void endOperations();

    @Override
    public void runOpMode() {
        initialise();

        waitForStart();

        runOperations();

        endOperations();
    }

    //************
    //INITIALIZARE
    //************

    protected void initialise()
    {
        //hardware mapping
        Motor_FL = hardwareMap.dcMotor.get("Motor_FL");
        Motor_FR = hardwareMap.dcMotor.get("Motor_FR");
        Motor_BL = hardwareMap.dcMotor.get("Motor_BL");
        Motor_BR = hardwareMap.dcMotor.get("Motor_BR");

        servo_L = hardwareMap.servo.get("servo_L");
        servo_R = hardwareMap.servo.get("servo_R");

        color = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color");
        RangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeL");
        RangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeR");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        //setare directii
        Motor_BL.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_FL.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor_BR.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_FR.setDirection(DcMotorSimple.Direction.FORWARD);

        //setare
        Motor_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initializare putere
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        //setare color
        color.enableLed(true);

        //calibrare gyro
        gyro.calibrate();
        while (gyro.isCalibrating()){
            idle();
        }
    }

    //************
    //MISCARE
    //************

    //Set the motors power individually
    protected void SetWheelsPower(double FL, double FR, double BL, double BR){
        Motor_FR.setPower(FR);
        Motor_BL.setPower(BL);
        Motor_FL.setPower(FL);
        Motor_BR.setPower(BR);
    }

    //Overcharge of the previous function, takes only two arguments and sets the wheels power based
    //on mecanum wheel dependence
    protected void SetWheelsPower(double FLBR, double FRBL){
        Motor_FR.setPower(FRBL);
        Motor_BL.setPower(FRBL);
        Motor_FL.setPower(FLBR);
        Motor_BR.setPower(FLBR);
    }

    //Provided with the speed and the angle in degrees(relative to the current rotation), make the
    //robot pan-move in that direction
    protected void WalkAtAngle(double speed, double angle){
        //Transform from angle to vectorial distribution(0 is 0%, 1 is 100%, you could think of it
        //as the proportion of X and Y)
        double AxisXVector = Math.cos(Math.toRadians(angle));
        double AxisYVector = Math.sin(Math.toRadians(angle));

        //Power the wheel motors according to the vectorial distribution
        //The function that describes the vectorial distribution is
        double VectorFLBR = MecanumFunctionCalculator(AxisXVector, AxisYVector, true);
        double VectorFRBL = MecanumFunctionCalculator(AxisXVector, AxisYVector, false);

        //Scale that vector by the desired speed, keeping in mind the maximum
        speed = Range.clip(speed, 0, 1);
        double ScalingCoeficient = speed/Math.max(VectorFLBR, VectorFRBL);

        double SpeedFLBR = VectorFLBR * ScalingCoeficient;
        double SpeedFRBL = VectorFRBL * ScalingCoeficient;


        SetWheelsPower(SpeedFLBR, SpeedFRBL);
    }

    protected void WalkEncoder(double dist , double angle){
        WalkAtAngle(0.7, angle);

        //calculate vectorials
        double TargetXVector = dist * Math.cos(angle);
        double TargetYVector = dist * Math.sin(angle);

        //calculate the encoder target
        double TargetFLBR = MecanumFunctionCalculator(TargetXVector, TargetYVector, true);
        double TargetFRBL = MecanumFunctionCalculator(TargetXVector, TargetYVector, false);

        //making run to position by hand, because the normal function sometimes has bugs
        while(Math.abs(TargetFLBR) > Math.abs(Motor_FL.getCurrentPosition()) || Math.abs(TargetFRBL) > Math.abs(Motor_FR.getCurrentPosition())){
            idle();
        }

        StopMotors();

        //TODO: daca merge sunt zeu
    }

    //need comment
    protected void WalkObstacleAndRangeNORMAL(double distanceFromWall, boolean bStartAllignedWithWall){
        //se aliniaza cu zidul din fata
        if(bStartAllignedWithWall){
            while(Math.abs(RangeL.rawUltrasonic() - RangeL.rawUltrasonic()) < 10/*magic number*/){
                int dir = (int)Math.signum(RangeL.rawUltrasonic() - RangeL.rawUltrasonic());
                SetWheelsPower(-0.2 * dir, 0.2 * dir, -0.2 * dir, 0.2 * dir);
            }
        }

        final double target = distanceFromWall;
        final double delay  = 2000;
        final long period = 10L; //while ce opereaza la frecventa de 10 ms

        boolean bIsUsingEncoder = false;

        ///IMPLEMENTARE GYRO
        gyro.resetZAxisIntegrator();
        double currentHeading = gyro.getHeading();

        ///TEST**************
        double pGain = 1/(target - 5); //daca zidul sau alt robot se apropie mai mult decat trebuie atunci sa mearga la viteza maxima in spate
        double dGain = 0.0;

        double errorRight = target - RangeL.getDistance(DistanceUnit.CM);
        double errorLeft = target - RangeL.getDistance(DistanceUnit.CM);

        double proportionalSpeedLeft = 0;
        double proportionalSpeedRight = 0;

        double finalSpeedLeft = 0, finalSpeedRight = 0;

        double initValueL = 0, initValueR = 0;

        float steadyTimer = 0;

        while(opModeIsActive() && steadyTimer < delay){

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
            if(Math.abs(finalSpeedLeft) < TOLERANCE ){
                finalSpeedLeft = 0;
            }
            if(Math.abs(finalSpeedRight) < TOLERANCE ){
                finalSpeedRight = 0;
            }

            //daca robotul trebuie sa se intoarca
            if(finalSpeedLeft > 0 && finalSpeedRight < 0){
                finalSpeedLeft = 0;
            }
            if(finalSpeedRight > 0 && finalSpeedLeft < 0){
                finalSpeedRight = 0;
            }

            //****************************
            //gyro
            currentHeading = gyro.getHeading();
            if(currentHeading > 180){
                currentHeading = currentHeading - 360;
            }
            if(Math.abs(currentHeading) > 2 && !bIsUsingEncoder){
                bIsUsingEncoder = true;
                initValueL = Motor_FL.getCurrentPosition();
                initValueR = Motor_FR.getCurrentPosition();
            }

            //****************************
            //retrack
            if(bIsUsingEncoder){
                if(initValueL <= Motor_FL.getCurrentPosition()){
                    finalSpeedLeft = 0;
                }
                if(initValueR <= Motor_FR.getCurrentPosition()){
                    finalSpeedRight = 0;
                }

                if(initValueL < Motor_FL.getCurrentPosition() && initValueR < Motor_FR.getCurrentPosition()){
                    bIsUsingEncoder = false;
                    gyro.resetZAxisIntegrator();
                }
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

        ///TEST END*****************
        StopMotors();
    }

    //opresc toate motoarele
    protected void StopMotors(){
        Motor_BL.setPower(0);
        Motor_BR.setPower(0);
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
    }

    //ma rotesc la angle grade: negativ e pentru right, pozitiv e pentru left;
    protected void Rotate(double angle){
        boolean bAngleIsNegative = false;
        double FinalAngle = gyro.getHeading()+angle;

        if ( FinalAngle < 0 ) {
            FinalAngle += 360;
            bAngleIsNegative = true;
        }

        double FL_Power = 0.7;
        double BL_Power = 0.7;
        double FR_Power = -0.7;
        double BR_Power = -0.7;

        if ( bAngleIsNegative ) {
            FL_Power *= -1;
            BL_Power *= -1;
            FR_Power *= -1;
            BR_Power *= -1;
        }

        Motor_FL.setPower(FL_Power);
        Motor_BL.setPower(BL_Power);
        Motor_FR.setPower(FR_Power);
        Motor_BR.setPower(BR_Power);

        while ( opModeIsActive() && Math.abs(FinalAngle-angle) > 5 ) {
            idle();
        }

        RotateSlowly(angle, FL_Power/2, BL_Power/2, FR_Power/2, BR_Power/2);
        StopMotors();
    }

    protected void RotateSlowly(double angle, double FL_Power, double BL_Power, double FR_Power, double BR_Power){
        double FinalAngle = gyro.getHeading()+angle;

        Motor_FL.setPower(FL_Power);
        Motor_BL.setPower(BL_Power);
        Motor_FR.setPower(FR_Power);
        Motor_BR.setPower(BR_Power);

        while ( opModeIsActive() && Math.abs(FinalAngle-angle) > 0 ) {
            idle();
        }
    }

    //ma aliniez cu peretele la distanta distance
    protected void AlignWithWall(double distance){
        double FL_Power = 0.7;
        double BL_Power = 0.7;
        double FR_Power = -0.7;
        double BR_Power = -0.7;

        Motor_FL.setPower(FL_Power);
        Motor_BL.setPower(BL_Power);
        Motor_FR.setPower(FR_Power);
        Motor_BR.setPower(BR_Power);

        while ( opModeIsActive() && Math.abs(RangeL.getDistance(DistanceUnit.CM) - RangeR.getDistance(DistanceUnit.CM)) > 5 ) {
            idle();
        }

        AlignWithWallSlowly(distance, FL_Power/2, BL_Power/2, FR_Power/2, BR_Power/2);
        StopMotors();

    }

    protected void AlignWithWallSlowly(double distance, double FL_Power, double BL_Power, double FR_Power, double BR_Power){
        Motor_FL.setPower(FL_Power);
        Motor_BL.setPower(BL_Power);
        Motor_FR.setPower(FR_Power);
        Motor_BR.setPower(BR_Power);

        while ( opModeIsActive() && Math.abs(RangeL.getDistance(DistanceUnit.CM) - RangeR.getDistance(DistanceUnit.CM)) > 0 ) {
            idle();
        }
    }

    //************
    //MISCELLANEOUS
    //************

    protected double MecanumFunctionCalculator(double VectorX, double VectorY, boolean bFLBR){
        return bFLBR? (Math.signum(VectorY) * Math.pow(VectorY, 2)) - (Math.signum(VectorX) * Math.pow(VectorX, 2)) : (Math.signum(VectorY) * Math.pow(VectorY, 2)) + (Math.signum(VectorX) * Math.pow(VectorX, 2));
    }

}