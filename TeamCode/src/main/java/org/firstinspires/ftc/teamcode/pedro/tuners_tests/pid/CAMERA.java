package org.firstinspires.ftc.teamcode.pedro.tuners_tests.pid;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Extendo;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Slides;
import org.firstinspires.ftc.teamcode.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedro.constants.LConstants;

import java.util.List;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "CAMERA", group = "Examples")
public class CAMERA extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private Limelight3A limelight;

    Servos servos = null;
    Slides slides = null;
    Extendo extendo = null;
    HardwareClass hardwareClass = null;

    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;

    public static int smallDelay = 200, bigDealy = 500;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    double x = 0 , cos = 0 , sin = 0 , ClampPosition = 0 , ClampPositionLeft = 0, Close = -1;
    double y = 0 , max=0 , FLPower = 0 , FRPower = 0, BLPower = 0 , BRPower = 0;
    double turn = 0 , theta = 0 , power = 0 , Trig_Forc = 0;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(37.5, 11.01, Math.toRadians(0));

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */


    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                find();
                setPathState(-1);
                break;
        }
    }

    public void Drive(double power){
        FLPower = power;
        FRPower = -power;
        BLPower = -power;
        BRPower = power;

        FL.setPower(FLPower);
        FR.setPower(FRPower);
        BL.setPower(BLPower);
        BR.setPower(BRPower);
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        servos = Servos.getInstance(hardwareMap,telemetry);
        slides = Slides.getInstance(hardwareMap,telemetry);
        extendo = Extendo.getInstance(hardwareMap,telemetry);
        hardwareClass = HardwareClass.getInstance(hardwareMap);

        slides.setup();
        servos.setup();
        extendo.setup();

        servos.start();
        servos.stop();

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        hardwareClass.LS.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        servos.camOut();

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public void find(){
        int v = 1;
        while (v == 1){
            LLResult result = limelight.getLatestResult();
            servos.intake();
            extendo.goToPosition(200);
            servos.wait(100);
            if (result.isValid()){
                double extendoPose;
                result = limelight.getLatestResult();
                extendoPose = convertToNewRange(result.getTx() , -23 , 23 , 300 , 900);
                while (result.getTy() < -2 || result.getTy() > 2){
                    result = limelight.getLatestResult();
                    Drive(-result.getTy() / 100 * 7);
                }
                extendo.goToPosition(extendoPose);
                angle(result);
                delay(500);
                servos.align();
                servos.wait(100);
                servos.transferSpec();
                delay(100);
                if(hardwareClass.IntakeSensor.getDistance(DistanceUnit.MM) < 56 && hardwareClass.IntakeSensor.getDistance(DistanceUnit.MM) != 0){
                    v = 0;
                    servos.camIn();
                }
            }
        }
    }

    public void angle(LLResult result){
        double width = 0 , height = 0;
        double Mx = -999 , mx = 999, My = -999, my = 999;
        double i = 0, Iy = 0;

        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
        for (LLResultTypes.ColorResult cr : colorResults) {
            telemetry.addData("corners: ",  cr.getTargetCorners());
            for(List<Double> point : cr.getTargetCorners()){
                double x = point.get(0);
                double y = point.get(1);
                telemetry.addLine("Corner: " + "X:" + x +"; Y:" + y + "; Index: " + i);

                if(Mx < x){ Mx = x;}
                if(My < y){ My = y; Iy = i;}
                if(mx > x) mx = x;
                if(my > y) my = y;
                i++;
            }
        }

        width = Math.abs(Mx - mx);
        height = Math.abs(My - my);

        if(height > width - 20){
            servos.Pivot.setPosition(hardwareClass.PIVOT_PERPENDICULAR);
        }
        else if(width - 20 > height){
            servos.Pivot.setPosition(hardwareClass.PIVOT_MAX_RIGHT);
        }
    }

    public double convertToNewRange(double value, double oldMin, double oldMax, double newMin, double newMax){
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }

    public void giveSpecimen(){
        servos.wait(50);
        servos.align();
        servos.transfer();
        servos.wait(100);
        extendo.goToPosition(hardwareClass.IN);
        servos.adjust();
        servos.wait(300);
        servos.outtakeTransf();
        OuttakeSample();
    }

    private void delay(int delay){
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private void OuttakeSample(){
        servos.outtake();
        slides.setCoefs(0.01,0,0);
        slides.goToPosition(hardwareClass.HIGH_BASKET);
    }

    private void slidesBackInRobot(){
        slides.setCoefs(0.002,0,0);
        slides.goToPosition(hardwareClass.IN_ROBOT);
    }

    private void extendAndPivot(int distance , float angle){
        extendo.goToPosition(distance);
        servos.rotatePivot(angle);
    }

    public void giveSpecimen3(){
        servos.wait(500);
        servos.align();
        servos.transferSpec();
        servos.wait(200);
        extendo.goToPosition(hardwareClass.IN);
        servos.adjust();
        servos.wait(300);
    }
}

