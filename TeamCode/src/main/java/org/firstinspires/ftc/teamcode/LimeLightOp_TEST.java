package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
//@TeleOp
public class LimeLightOp_TEST extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("Active",result.isValid());
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    List<List<Double>> corners = fiducial.getTargetCorners();
                    telemetry.addData("corners",corners.size());
                    double angle = computeAngle(corners);
                    telemetry.addData("Angle to Target:", angle);
                }
                telemetry.update();
            }
        }
    }

    private double computeAngle(List<List<Double>> corners) {
        // Get the center of the rectangle
        double centerX = (corners.get(0).get(0) + corners.get(1).get(0) + corners.get(2).get(0) + corners.get(3).get(0)) / 4.0;

        // Assume limelight's FOV and resolution (adjust as needed)
        double limelightFOV = 54.0;  // Horizontal FOV in degrees
        double imageWidth = 640;   // Resolution width in pixels

        // Compute the angle based on displacement from center
        double deltaX = centerX - (imageWidth / 2.0);
        return (deltaX / (imageWidth / 2.0)) * (limelightFOV / 2.0);
    }
}