package org.firstinspires.ftc.teamcode.PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;


public class HangPID implements Runnable{

    private volatile boolean running = false;
    private String threadName = null;
    private String motorName = null;
    private Telemetry telemetry;
    private Thread thread = null;

    private double Kp, Ki, Kd;
    private double reference, current = 0;
    private double integralSum, derivative;
    private double error, lastError;
    private DcMotorEx motor1;

    double repetitions=0;

    private static HangPID single_instance = null;

    private int activePower = 1;
    int stop = 1;

    /*
    Explicit 'this' to state object type clearly
     */

    /**
     * @param motor1 - the motor to which the PID controller affects
     * @param telemetry - telemtry object for logging
     */
    public HangPID(DcMotorEx motor1, String motorName, String threadName, Telemetry telemetry){
        this.motor1 = motor1;
        this.motorName = motorName;
        this.threadName = threadName;
        this.telemetry = telemetry;
    }

    public HangPID(DcMotorEx motor1,
                   double kp,
                   double ki,
                   double kd,
                   double reference,
                   String motorName,
                   String threadName,
                   Telemetry telemetry){
        this.motor1 = motor1;
        this.Kp = kp;
        this.Ki = ki;
        this.Kd = kd;
        this.motorName = motorName;
        this.threadName = threadName;
        this.telemetry = telemetry;
    }
    public boolean start(){
        try {
            if (this.thread == null || !this.thread.isAlive()) {
                telemetry.addLine("Trying to start");
                telemetry.update();
                this.thread = new Thread(this);
                this.running = true;
                this.current = motor1.getCurrentPosition();
                this.repetitions=0;
                this.thread.start();

                return true;
            } else{
                return false;
            }
        }catch (IllegalThreadStateException e){
            this.telemetry.addData(this.threadName + " -> thread start error: ",e);
            telemetry.update();
            return false;
        }
    }

    public boolean status(){
        return running;
    }

    public boolean stop(){
        try {
            if (this.thread != null) {
                this.running = false;
                repetitions = 0;
                this.thread.join();
                this.thread = null;
                this.telemetry.addData(this.threadName + " -> thread stop error", "is null");
                return true;
            } else {
                this.telemetry.addData(this.threadName + " -> thread stop error", "is null");
                telemetry.update();
                return false;
            }
        } catch (InterruptedException e) {
            this.telemetry.addData(this.threadName + " -> thread stop error", e);
            telemetry.update();
            return false;
        }
    }

    /**
     * @param p - kp
     * @param i - ki
     * @param d - kd
     */
    public void setCoefficients(double p, double i, double d){
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
    }

    /**
     * @param reference - desired position
     */
    public void setReference(double reference){
        this.reference = reference;
    }

    public void delay(int sec) throws  InterruptedException{
        this.thread.wait(sec);
    }

    public void stopPID(){
        stop = 0;
    }

    public void startPID(){
        stop = 1;
    }
    
    FtcDashboard ftdb = FtcDashboard.getInstance();
    
    @Override
    public void run() {
        // short amount of time resets in every loop so every point of the
        while(running) {
            ElapsedTime timer = new ElapsedTime();
            while ((Math.abs(error) > 10 || repetitions < 40) && stop == 1) {
        
                current = motor1.getCurrentPosition();
        
                error = reference - current;
                derivative = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());


                double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
                
                motor1.setPower(output * activePower);
        
                telemetry.addData(motorName + " target", reference);
                telemetry.addData(motorName + " pos", current);
                
                TelemetryPacket tp = new TelemetryPacket();
                tp.put("pos", current);
                tp.put("ref", reference);
                tp.put("error", error);
                ftdb.sendTelemetryPacket(
                        tp
                );
        
                telemetry.update();
                
                lastError = error;
                timer.reset();
            }
            current = motor1.getCurrentPosition();
            telemetry.addData(motorName + " pos", current);
            telemetry.update();
            repetitions++;
        }
    }

    public void activetePower(){
        activePower = 1;
    }

    public void deactivatePower(){
        activePower = 0;
    }

    public static synchronized HangPID getInstance(DcMotorEx motor1, String motorName, String threadName, Telemetry telemetry){
        if(single_instance == null){
            single_instance = new HangPID(motor1,motorName  , threadName,telemetry);
        }
        return single_instance;
    }

}