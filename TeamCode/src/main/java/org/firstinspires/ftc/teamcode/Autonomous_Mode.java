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
    protected final double deadzone = 0.1;

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
        //TODO - WalkEncoder : mers distanta dist la unghiul angle
        WalkAtAngle(0.7, angle);

        //calculate vectorials
        double TargetXVector = dist * Math.cos(angle);
        double TargetYVector = dist * Math.sin(angle);

        //calculate the encoder target
        double TargetFLBR = MecanumFunctionCalculator(TargetXVector, TargetYVector, true);
        double TargetFRBL = MecanumFunctionCalculator(TargetXVector, TargetYVector, false);

        //TODO: stii tu ce trebuie sa mai faci
    }

    //merg pana la un obiect
    protected boolean WalkUntilObject(double angle){
        WalkAtAngle(0.2, angle);

        while ( opModeIsActive() && color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 0 ){
            idle();
        }
        StopMotors();

        //daca e bun, il mut
        if ( GoodColor() ){
            WalkEncoder(5, 0);
            WalkEncoder(-5, 0);
            return true;
        }

        return false;
    }

    //opresc toate motoarele
    protected void StopMotors(){
        Motor_BL.setPower(0);
        Motor_BR.setPower(0);
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
    }

    //ma rotesc la angle grade; negativ e pentru right, pozitiv e pentru left;
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

    //verific obiectele pana dau de cel bun
    protected void FindCube(){

        if (WalkUntilObject(0)) {
            return;
        }
        if (WalkUntilObject(90)) {
            return;
        }

        WalkUntilObject(-90);
        return;
    }

    //citesc culoarea
    protected boolean GoodColor(){
        int curColor = color.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
        if ( curColor >= 8 && curColor <= 10 ) return true;
        return false;
    }

    protected double MecanumFunctionCalculator(double VectorX, double VectorY, boolean bFLBR){
        return bFLBR? (Math.signum(VectorY) * Math.pow(VectorY, 2)) - (Math.signum(VectorX) * Math.pow(VectorX, 2)) : (Math.signum(VectorY) * Math.pow(VectorY, 2)) + (Math.signum(VectorX) * Math.pow(VectorX, 2));
    }

}