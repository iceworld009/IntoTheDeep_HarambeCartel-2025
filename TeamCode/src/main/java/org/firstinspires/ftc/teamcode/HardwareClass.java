package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HardwareClass {

    //DcMotor

    public int CAMERA_MAX_RIGHT = 0;
    public int CAMERA_MAX_LEFT = 0;
    public int ROBOT_MAX_LEFT = 0;
    public int ROBOT_MAX_RIGHT = 0;

    public DcMotorEx Push;

    public DcMotor FR, FL , BR , BL;
    public DcMotorEx LS , RS, Extendo;
    public double SLIDES_MAX_POWER = 1;

    /** SLIDES PID VALUES */

    public double IN_ROBOT = 0, HIGH_BASKET = 890 ,LOW_BASKET = 350, HIGH_S_BASKET = 800;
    public double PREP_SPECIMEN = 330;
    public double PLACE_SPECIMEN = 330;

    public double HATZ_SPECIMEN = 90;

    /** AUTO COORDS*/

    public double X = 0 , Y = 0, HEAD = 0;

    /**  EXTENDO PID VALUES  */

    public double IN = 200, MAX = 1200 ;
    public double CLOSE = 300;
    public ColorRangeSensor IntakeSensor;

    //Servo
    public Servo ClawIn , ClawOut , IntakeRotate, OuttakeRotate, Pivot, MobiDick , Cam;
    // Intake claw
    public double CLAW_IN_CLOSED = 0.5 ,CLAW_IN_OPEN = 0.06 , CLAW_IN_ADJUST = 0.43;
    public double SPEC_CLAW_OPEN = 0.25 ,SPEC_CLAW_CLOSE = 0.5;

    public double MD_OUT = 0.13 ,MD_IN = 0.43;
    public double CAM_OUT = 0.47 ,CAM_IN = 0.13;
    // Intake rotation
    public double INTAKE_ROTATION_CORRECT = 0.17, INTAKE_ROTATION_DOWN = 0.58 , INTAKE_ROTATION_UP = 0 , INTAKE_ROTATION_PREP = 0.42,INTAKE_ROTATION_SEE = 0.39;
    // Intake Pivot
    public double PIVOT_MAX_LEFT = 0.8, PIVOT_MAX_RIGHT = 0.3 , PIVOT_RIGHT_TOP = 0.4 , PIVOT_LEFT_TOP = 0.1 , PIVOT_PERPENDICULAR = 0.54;
    public double CLAW_OUT_CLOSED = 0.62,CLAW_OUT_OPEN = 0.40, CLAW_OUT_SPEC = 0.57;
    // Outtake claw rotation
    public double OUTTAKE_ROTATION_TAKE = 0.07, OUTTAKE_ROTATION_PLACE = 0.9, OUTTAKE_ROTATION_VERTICAL = 0.63 , OUTTAKE_ROTATION_SPEC = 0.95   , OUTTAKE_ROTATION_HIGH_VERT = 0.62,OUTTAKE_ROTATION_TAKE_LOW = 0.06;
    //Specimen
    public double SPECIMEN_CLOSED = 0.53 , SPECIMEN_OPEN = 0.3;

    //Singleton
    private static HardwareClass hardwareClass = null;

    public HardwareClass(HardwareMap hardwareMap){
        //Chassy
        this.FR = hardwareMap.get(DcMotor.class , "FR");
        this.FL = hardwareMap.get(DcMotor.class , "FL");
        this.BR = hardwareMap.get(DcMotor.class , "BR");
        this.BL = hardwareMap.get(DcMotor.class , "BL");

        this.Push = hardwareMap.get(DcMotorEx.class , "PUSH");
        //Slides
        this.RS = hardwareMap.get(DcMotorEx.class,"RS");
        this.LS = hardwareMap.get(DcMotorEx.class,"LS");
        this.Extendo = hardwareMap.get(DcMotorEx.class , "EX");
        //Servos
        this.ClawIn = hardwareMap.get(Servo.class, "CI");
        this.ClawOut = hardwareMap.get(Servo.class, "CO");
        this.IntakeRotate = hardwareMap.get(Servo.class, "RI");
        this.OuttakeRotate = hardwareMap.get(Servo.class, "RO");
        this.Cam = hardwareMap.get(Servo.class, "CAM");
        this.Pivot = hardwareMap.get(Servo.class, "PV");
        this.MobiDick = hardwareMap.get(Servo.class, "MD");

        this.IntakeSensor = hardwareMap.get(ColorRangeSensor.class, "IS");

    }

    public static synchronized HardwareClass getInstance(HardwareMap hardwareMap){
        if(hardwareClass == null)
            hardwareClass = new HardwareClass(hardwareMap);
        return hardwareClass;
    }
}
