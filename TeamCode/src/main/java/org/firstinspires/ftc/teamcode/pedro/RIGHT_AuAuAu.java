package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Extendo;
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

@Autonomous(name = "RIGHT_5", group = "Examples")
public class RIGHT_AuAuAu extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    Servos servos = null;
    Slides slides = null;
    Extendo extendo = null;
    HardwareClass hardwareClass = null;

    public static int smallDelay = 200, bigDealy = 500;

    private ElapsedTime time;

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
    private final Pose startPose = new Pose(74, 11, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(70, 42, Math.toRadians(270));
    private final Pose takePose = new Pose(112, 12, Math.toRadians(270));
    private final Pose takePose1 = new Pose(112, 10.5, Math.toRadians(270));

    private final Pose PREPtakePose = new Pose(112, 25, Math.toRadians(270));

    private final Pose pickup1 = new Pose(100, 35, Math.toRadians(270));
    private final Pose pickup2 = new Pose(111, 46, Math.toRadians(270));
    private final Pose pickup3 = new Pose(112, 30, Math.toRadians(270));
    private final Pose pickup4 = new Pose(124, 46, Math.toRadians(270));
    private final Pose pickup5 = new Pose(125, 30, Math.toRadians(270));
    private final Pose pickup6 = new Pose(130, 46, Math.toRadians(270));
    private final Pose pickup7 = new Pose(131, 30, Math.toRadians(270));

    private final Pose basket = new Pose(15, 16, Math.toRadians(45));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;

    private PathChain grab1 , grab2 , grab3 , prepSpec , scoreSpec , takeSpec;
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
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1.getHeading())
                .addParametricCallback(0 , () -> {
                    servos.outtakePlace();
                    servos.outtakeReturn();
                    slides.goToPosition(hardwareClass.IN_ROBOT);
                    servos.outtakeVERYDown();
                })
                .addPath(new BezierLine(new Point(pickup1), new Point(pickup2)))
                .setLinearHeadingInterpolation(pickup1.getHeading(), pickup2.getHeading())
                .addPath(new BezierLine(new Point(pickup2), new Point(pickup3)))
                .setLinearHeadingInterpolation(pickup2.getHeading(), pickup3.getHeading())
                .addParametricCallback(0 , () -> {
                    hardwareClass.MobiDick.setPosition(hardwareClass.MD_OUT);
                })
                
                .addPath(new BezierLine(new Point(pickup3), new Point(pickup4)))
                .setLinearHeadingInterpolation(pickup3.getHeading(), pickup4.getHeading())
                .addParametricCallback(0 , () -> {
                    hardwareClass.MobiDick.setPosition(hardwareClass.MD_IN);
                })
                .addPath(new BezierLine(new Point(pickup4), new Point(pickup5)))
                .setLinearHeadingInterpolation(pickup4.getHeading(), pickup5.getHeading())
                .addParametricCallback(0 , () -> {
                    hardwareClass.MobiDick.setPosition(hardwareClass.MD_OUT);
                })

                .addPath(new BezierLine(new Point(pickup5), new Point(pickup6)))
                .setLinearHeadingInterpolation(pickup5.getHeading(), pickup6.getHeading())
                .addParametricCallback(0 , () -> {
                    hardwareClass.MobiDick.setPosition(hardwareClass.MD_IN);
                })
                .addPath(new BezierLine(new Point(pickup6), new Point(pickup7)))
                .setLinearHeadingInterpolation(pickup6.getHeading(), pickup7.getHeading())
                .addParametricCallback(0 , () -> {
                    hardwareClass.MobiDick.setPosition(hardwareClass.MD_OUT);
                })

                .addPath(new BezierLine(new Point(pickup7), new Point(takePose1)))
                .setLinearHeadingInterpolation(pickup7.getHeading(), takePose1.getHeading())
                .build();

        scoreSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(takePose), new Point(scorePose)))
                .setLinearHeadingInterpolation(takePose.getHeading(), scorePose.getHeading())

                .addParametricCallback(0 , () -> {
                    hardwareClass.MobiDick.setPosition(hardwareClass.MD_IN);
                    servos.outtakeSpec();

                })
                .addParametricCallback(0.1 , () -> {
                    slides.goToPosition(hardwareClass.PREP_SPECIMEN);
                })

                .addPath(new BezierLine(new Point(scorePose), new Point(PREPtakePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PREPtakePose.getHeading())

                .addParametricCallback(0 , () -> {
                    servos.outtakePlace();
                    servos.outtakeReturn();
                    slides.goToPosition(hardwareClass.IN_ROBOT);
                    servos.outtakeVERYDown();
                })
                .addPath(new BezierLine(new Point(PREPtakePose), new Point(takePose)))
                .setLinearHeadingInterpolation(PREPtakePose.getHeading(), takePose.getHeading())
                .addParametricCallback(0.95 , () -> {
                    servos.outtakeSpec();
                })

                .addPath(new BezierLine(new Point(takePose), new Point(scorePose)))
                .setLinearHeadingInterpolation(takePose.getHeading(), scorePose.getHeading())

                .addParametricCallback(0 , () -> {
                    servos.outtakeSpec();
                })
                .addParametricCallback(0.1 , () -> {
                    slides.goToPosition(hardwareClass.PREP_SPECIMEN);
                })

                .addPath(new BezierLine(new Point(scorePose), new Point(PREPtakePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PREPtakePose.getHeading())

                .addParametricCallback(0 , () -> {
                    servos.outtakePlace();
                    servos.outtakeReturn();
                    slides.goToPosition(hardwareClass.IN_ROBOT);
                    servos.outtakeVERYDown();
                })
                .addPath(new BezierLine(new Point(PREPtakePose), new Point(takePose)))
                .setLinearHeadingInterpolation(PREPtakePose.getHeading(), takePose.getHeading())
                .addParametricCallback(0.95 , () -> {
                    servos.outtakeSpec();
                })

                .addPath(new BezierLine(new Point(takePose), new Point(scorePose)))
                .setLinearHeadingInterpolation(takePose.getHeading(), scorePose.getHeading())

                .addParametricCallback(0 , () -> {
                    servos.outtakeSpec();
                })
                .addParametricCallback(0.1 , () -> {
                    slides.goToPosition(hardwareClass.PREP_SPECIMEN);
                })

                .addPath(new BezierLine(new Point(scorePose), new Point(PREPtakePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PREPtakePose.getHeading())

                .addParametricCallback(0 , () -> {
                    servos.outtakePlace();
                    servos.outtakeReturn();
                    slides.goToPosition(hardwareClass.IN_ROBOT);
                    servos.outtakeVERYDown();
                })
                .addPath(new BezierLine(new Point(PREPtakePose), new Point(takePose)))
                .setLinearHeadingInterpolation(PREPtakePose.getHeading(), takePose.getHeading())

                .addParametricCallback(0.95 , () -> {
                    servos.outtakeSpec();
                })

                .addPath(new BezierLine(new Point(takePose), new Point(scorePose)))
                .setLinearHeadingInterpolation(takePose.getHeading(), scorePose.getHeading())

                .addParametricCallback(0 , () -> {
                    servos.outtakeSpec();
                })
                .addParametricCallback(0.1 , () -> {
                    slides.goToPosition(hardwareClass.PREP_SPECIMEN);
                })

                .addPath(new BezierLine(new Point(scorePose), new Point(PREPtakePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PREPtakePose.getHeading())

                .addParametricCallback(0 , () -> {
                    servos.outtakePlace();
                    servos.outtakeReturn();
                    slides.goToPosition(hardwareClass.IN_ROBOT);
                    servos.outtakeVERYDown();
                })
                .addPath(new BezierLine(new Point(PREPtakePose), new Point(takePose)))
                .setLinearHeadingInterpolation(PREPtakePose.getHeading(), takePose.getHeading())

                .addParametricCallback(0.95 , () -> {
                    servos.outtakeSpec();
                })
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                time.startTime();
                servos.outtakeSpec();
                servos.camIn();
                hardwareClass.IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_PREP);
                delay(200);
                slides.goToPosition(hardwareClass.PREP_SPECIMEN);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
               if(!follower.isBusy()) {
                    follower.followPath(grab1,false);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSpec,true);
                    servos.outtakePlace();
                    servos.outtakeReturn();
                    slides.goToPosition(hardwareClass.IN_ROBOT);
                    setPathState(3);
                }
            case 3:
                if(!follower.isBusy()) {
                    servos.outtakePlace();
                    servos.outtakeReturn();
                    slides.goToPosition(hardwareClass.IN_ROBOT);
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
        extendo = Extendo.getInstance(hardwareMap,telemetry);
        hardwareClass = HardwareClass.getInstance(hardwareMap);

        slides.setup();
        servos.setup();
        extendo.setup();

        servos.start();
        servos.stop();

        hardwareClass.LS.setDirection(DcMotor.Direction.REVERSE);

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

    public void giveSpecimen(){
        servos.wait(50);
        servos.align();
        servos.transfer();
        servos.wait(100);
        extendo.goToPosition(hardwareClass.IN);
        servos.adjust();
        servos.wait(300);
        servos.outtakeTransf();
        hardwareClass.OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_PLACE - 0.2);
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
        slides.setCoefs(0.0015,0,0);
        slides.goToPosition(hardwareClass.IN_ROBOT);
    }

    private void extendAndPivot(int distance , float angle){
        extendo.goToPosition(distance);
        servos.intake();
        servos.rotatePivot(angle);
    }

    public void giveSpecimen3(){
        servos.wait(200);
        servos.align();
        servos.transferSpec();
        servos.wait(200);
        extendo.goToPosition(hardwareClass.IN);
        servos.adjust();
    }
}

