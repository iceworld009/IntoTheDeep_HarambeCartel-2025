package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.PID.SlidesPID;

public class Slides {

    //Declarations
    DcMotorEx LS , RS;
    Gamepad gm;

    private double BLPower , BRPower , FRPower , FLPower;
    private double power , turn , max , x , y;
    private double sin , cos , theta;
    private double max_power;

    Telemetry telemetry = null;

    public static double kp = 0.01, ki = 0, kd = 0;
    public static double target = 100;
    SlidesPID generalPID = null;
    HardwareClass hardwareClass = null;

    //Singleton
    private static Slides single_instance = null;
    Thread thread = null;
    private boolean running = false;

    public Slides(HardwareClass hardwareClass, Gamepad gamepad, Telemetry telemetry){
        this.LS = hardwareClass.LS;
        this.RS = hardwareClass.RS;
        this.max_power = hardwareClass.SLIDES_MAX_POWER;
        this.gm = gamepad;
        this.telemetry = telemetry;
        this.hardwareClass = hardwareClass;
    }

    public Slides(HardwareClass hardwareClass, Telemetry telemetry){
        this.LS = hardwareClass.LS;
        this.RS = hardwareClass.RS;
        this.max_power = hardwareClass.SLIDES_MAX_POWER;
        this.telemetry = telemetry;
        this.hardwareClass = hardwareClass;
    }

    //Sare peste slideuri
    public void Jump()
    {
        goToPosition(HardwareClass.JumpHeight);
    }

    // IDK cuaie ;0
    public void nulpoz() {
        goToPosition(50);

    /*
    ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⢀⡠⠤⠒⠒⠲⠤⣤⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
            ⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠟⠁⠀⠀⠀⠀⠀⠀⠁⠙⡮⣂⠀⠀⠀⠀⠀⠀⠀⠀
            ⠀⠀⠀⠀⠀⠀⠀⣢⠊⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠚⣦⣂⠀⠀⠀⠀⠀⠀
            ⠀⠀⠀⠀⠀⣠⣝⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠱⣥⠄⠀⠀⠀⠀
            ⠀⠀⠀⠀⣸⠮⣶⠞⠛⣛⣛⣛⣷⠶⠀⠀⠐⢶⣟⣛⣛⡛⠓⠿⡚⣮⡀⠀⠀⠀
            ⠀⠀⢀⢪⠛⠁⠀⣠⣚⣶⣶⡶⡆⡗⠀⠀⠰⡇⣶⣿⣿⣟⡢⠄⠀⠙⣮⢄⠀⠀
            ⠀⢀⢦⡋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⣳⢆⠀
            ⠀⢨⡧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡤⠀⠀⠀⠈⢯⠄
            ⠀⡬⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣤⠀⡼⠁⠀⠀⠀⠀⠊⡄
            ⠀⡏⠀⠀⠀⠀⠀⠀⠀⠀⡦⠤⠤⠤⠤⠤⠤⠴⠒⠋⠁⢰⠇⠀⠀⠀⠀⠀⠀⡇
            ⠰⣳⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠚⠀⠀⠀⠀⠀⢀⣴⠛
            ⠀⠁⢷⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⢵⠙⠀
            ⠀⠀⠀⠀⠩⠗⠲⠤⣤⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣠⡤⠴⠶⠚⠋⠁⠀⠀⠀
            ⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠀⠀⠀⠀⠀⠀⠐⠂⠈⠀⠈⠉⠁⠀⠀⠀⠀⠀⠀⠀

     */
    }
    public void setup(){
        running = true;
        //LS.setDirection(DcMotor.Direction.FORWARD);
        RS.setDirection(DcMotor.Direction.FORWARD);

        RS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        generalPID = SlidesPID.getInstance(RS , LS , "","",telemetry);
        generalPID.setCoefficients(kp,ki,kd);
        generalPID.start();
    }

    public void setPOW(int input){
        RS.setPower(input);
        LS.setPower(input);
    }

    public void resetMotor(){

        generalPID.setReference(-100);
        RS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        generalPID.setReference(0);
        RS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void goToPosition(double target){
        generalPID.setReference(target);
    }

    public void setCoefs(double p , double i , double d){
        generalPID.setCoefficients(p , i , d);
    }

    public void stop(){
        generalPID.pause();
    }

    public boolean getStatus(){
        return running;
    }

    public static synchronized Slides getInstance(HardwareMap hardwareMap, Gamepad gm, Telemetry telemetry){
        if(single_instance == null){
            single_instance = new Slides(HardwareClass.getInstance(hardwareMap), gm, telemetry);
        }
        return single_instance;
    }

    public void delay(int dealy) throws InterruptedException {
        generalPID.wait(dealy);
    }

    public static synchronized Slides getInstance(HardwareMap hardwareMap, Telemetry telemetry){
        if(single_instance == null){
            single_instance = new Slides(HardwareClass.getInstance(hardwareMap), telemetry);
        }
        return single_instance;
    }
}
