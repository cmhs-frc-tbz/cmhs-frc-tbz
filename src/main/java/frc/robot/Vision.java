/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;

public class Vision {

    private static Vision instance;
    private static int[] red_tags = { 5, 10, 2 };
    private static int[] blue_tags = { 21, 26, 18 };

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    public static class targetYawAndRange {
        public double yaw;
        public double range;

        public targetYawAndRange(double yaw, double range) {
            this.yaw = yaw;
            this.range = range;
        }

    }

    private final PhotonCamera camera1;
    private final PhotonCamera camera2;

    // Simulation
    /**
     * @param estConsumer Lamba that will accept a pose estimate and pass it to your
     *                    desired {@link
     *                    edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */
    public Vision() {
        // this class should follow the singleton paradigm

        camera1 = new PhotonCamera(kCameraName);
        camera2 = new PhotonCamera(kCameraName);

    }

    public Double getPitch() {
        var results = camera1.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (Arrays.asList(red_tags).contains(target.getFiducialId())) {
                        return target.getYaw();
                    }
                }
            }
        }
        return null;

    }
    
    @SuppressWarnings("unlikely-arg-type")
    public boolean getTargets() {
        var results = camera1.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (Arrays.asList(red_tags).contains(target.getFiducialId())) {
                        return true;
                    }
                }
            }
        }
        return false;

    }


    public targetYawAndRange update() {
        boolean targetVisible = false;
        double targetYaw = 0.0;
        double targetRange = 0.0;
        var results = camera1.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (Arrays.asList(red_tags).contains(target.getFiducialId())) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                                0.5, // Measured with a tape measure, or in CAD.
                                1.435, // From 2024 game manual for ID 7
                                Units.degreesToRadians(25.94), // Measured with a protractor, or in CAD.
                                Units.degreesToRadians(target.getPitch()));

                        targetVisible = true;
                        return new targetYawAndRange(targetYaw, targetRange);
                    }
                }

            }
        }
        return null;

    }
}