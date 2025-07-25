package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Slides;
import org.firstinspires.ftc.teamcode.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedro.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "LEFT_4_TEST", group = "Examples")
public class LEFT_4_SIMPLE_TEST extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Limelight3A limelight;

    Servos servos = null;
    Slides slides = null;

    HardwareClass hardwareClass = null;

    public static int smallDelay = 200, bigDealy = 500;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(37.5, 11.01, Math.toRadians(90));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(15, 18, Math.toRadians(45));

    private final Pose scorePose1 = new Pose(17, 20, Math.toRadians(45));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(16.4, 22, Math.toRadians(72));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(16.4, 22, Math.toRadians(95));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(16, 24, Math.toRadians(118));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(38, 62, Math.toRadians(180));

    private final Pose parkPose2 = new Pose(30, 62, Math.toRadians(180));

    /* These are our Paths and PathChains that we will define in buildPaths() */

    private PathChain grab1 , grab2 , grab3 , parkPath , placeAndTake , takeFisrt, placeP, place1 , place2,place3 , park;
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

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

        placeP = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0 , () -> {
                    servos.outtakePlaceSample();
                })
                .build();

        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        place1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0 , () -> {
                    servos.outtakePlaceSample();
                })
                .build();

        grab2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0 , () -> {


                })
                .build();

        place2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0 , () -> {

                })
                .build();

        grab3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addParametricCallback(0 , () -> {


                })
                .build();

        place3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(parkPose2)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose2.getHeading())
                .addParametricCallback(0 , () -> {

                })
                .addPath(new BezierLine(new Point(parkPose2), new Point(parkPose)))
                .setLinearHeadingInterpolation(parkPose2.getHeading(), parkPose.getHeading())
                .addParametricCallback(0 , () -> {

                })
                .build();


        placeAndTake = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(parkPose2)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkPose2.getHeading())

                .addParametricCallback(0 , () -> {

                })

                .addPath(new BezierLine(new Point(parkPose2), new Point(scorePose1)))
                .setLinearHeadingInterpolation(parkPose2.getHeading(), scorePose1.getHeading())

                .addParametricCallback(0.1 , () -> {

                })

                .addParametricCallback(0.3 , () -> {

                })

                .addPath(new BezierLine(new Point(scorePose1), new Point(parkPose2)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), parkPose2.getHeading())
                .addParametricCallback(0 , () -> {

                })

                .addPath(new BezierLine(new Point(parkPose2), new Point(parkPose)))
                .setLinearHeadingInterpolation(parkPose2.getHeading(), parkPose.getHeading())
                .addParametricCallback(0 , () -> {

                })
                .build();

        placeAndTake = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(parkPose2)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkPose2.getHeading())

                .addParametricCallback(0 , () -> {

                })
                .build();

        takeFisrt = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(parkPose2)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose2.getHeading())
                .addParametricCallback(0 , () -> {

                })

                .addPath(new BezierLine(new Point(parkPose2), new Point(parkPose)))
                .setLinearHeadingInterpolation(parkPose2.getHeading(), parkPose.getHeading())
                .addParametricCallback(0 , () -> {

                })

                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                outtakePrep();
                follower.followPath(placeP);
                setPathState(1);
                break;
            case 1:
               if(!follower.isBusy()) {
                   servos.outtakeReturn();
                   slidesBackInRobot();
                   servos.intakeSweep();
                   follower.followPath(grab1,false);
                   delay(bigDealy);
                   servos.intakeIn();
                   setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    outtakePrep();
                    follower.followPath(place1,false);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(grab2,false);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(place2,false);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(grab3,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(place3,false);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(park,true);
                    setPathState(-1);
                }
                break;
        }
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
        hardwareClass = HardwareClass.getInstance(hardwareMap);

        slides.setup();
        servos.setup();
        servos.start();
        servos.stop();

        hardwareClass.LS.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

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



    private void delay(int delay){
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void find(){
        LLResult result = limelight.getLatestResult();
        result.getColorResults();
        double x = result.getTx();
        double y = result.getTy();

        double posX = convertToNewRange(x , -16 , 16 , -1.8 , 1.8) * 1.05;
        double posY = convertToNewRange(y , -10 , 10 , 0 , 5.4);

        Pose found = new Pose(follower.getPose().getX() + posY, follower.getPose().getY() - posX, follower.getPose().getHeading());
        PathChain foundPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(found)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), found.getHeading())
                .build();

        follower.followPath(foundPath,false);
    }

    public double convertToNewRange(double value, double oldMin, double oldMax, double newMin, double newMax){
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }

    private void slidesBackInRobot(){
        slides.setCoefs(0.002,0,0);
        slides.goToPosition(hardwareClass.IN_ROBOT);
    }

    void outtakePrep(){
        servos.transfer();
        //delay(70);
        servos.outtakePrepSample();
        slides.setCoefs(0.01,0,0);
        slides.goToPosition(HardwareClass.HIGH_BASKET);
    }

}

