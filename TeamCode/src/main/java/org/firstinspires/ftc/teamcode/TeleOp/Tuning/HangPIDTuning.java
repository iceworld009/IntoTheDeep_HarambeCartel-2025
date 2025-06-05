package org.firstinspires.ftc.teamcode.TeleOp.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PID.ExtendoPID;

//@TeleOp(name="Hang Pid Tuning", group = "")
@Config
public class HangPIDTuning extends LinearOpMode {
    
    DcMotorEx RightSlide = null;
    public static double kp = 0, ki = 0, kd = 0;
    public static double target = 100;
    ExtendoPID generalPID = null;
    
    @Override
    public void runOpMode(){
        
        // after INIT
        RightSlide = hardwareMap.get(DcMotorEx.class , "PUSH");

        //LeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        RightSlide.setDirection(DcMotor.Direction.REVERSE);
        RightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
 
        generalPID = new ExtendoPID(RightSlide  , "","",telemetry);
        generalPID.start();
        
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()){
            //What happens after START
            generalPID.setReference(target);
            generalPID.setCoefficients(kp,ki,kd);
            
        }

        // kp = 0.01 , kd = 0, ki = 0 || max = 900 , transfer = 50

        generalPID.stop();
        
    }
}