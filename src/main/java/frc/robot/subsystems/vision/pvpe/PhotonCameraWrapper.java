package frc.robot.subsystems.vision.pvpe;

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


 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.apriltag.AprilTagFields;
 import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants.VisionConstants;
 import java.io.IOException;
 import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
 
 public class PhotonCameraWrapper {
     PhotonCamera photonCamera;
     PhotonPoseEstimator photonPoseEstimator;

     public static final class VisionConstants {
        public static final String kPvpeCameraName = "apriltag0";
        public static final Transform3d robotToCam =
                new Transform3d(
                        new Translation3d(0.5, 0.0, 0.5),
                        new Rotation3d(
                                0, 0,
                                0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
      }
 
     public PhotonCameraWrapper() {
         // Change the name of your camera here to whatever it is in the PhotonVision UI.
         photonCamera = new PhotonCamera(VisionConstants.kPvpeCameraName);
 
         try {
             // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
             AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
             // Create pose estimator
             photonPoseEstimator =
                     new PhotonPoseEstimator(
                             fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, photonCamera, VisionConstants.robotToCam);
             photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
         } catch (IOException e) {
             // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
             // where the tags are.
             DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
             photonPoseEstimator = null;
         }
     }
 
     /**
      * @param estimatedRobotPose The current best guess at robot pose
      * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
      *     the estimate
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         if (photonPoseEstimator == null) {
            SmartDashboard.putBoolean("photonPoseEstimator is Null", true);
             // The field layout failed to load, so we cannot estimate poses.
             return Optional.empty();
         }
         SmartDashboard.putNumber("prevEstimatedRobotPose_X", prevEstimatedRobotPose.getX());
         SmartDashboard.putNumber("prevEstimatedRobotPose_Y", prevEstimatedRobotPose.getY());
         photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

         periodic(); // debugging

         return photonPoseEstimator.update();
     }

     public void periodic() {
        var result = photonCamera.getLatestResult();
        boolean limelightHasTargets = result.hasTargets();
        SmartDashboard.putBoolean("Target Present", limelightHasTargets);

        if (limelightHasTargets) { 
            PhotonTrackedTarget photonTrackedTarget = result.getBestTarget();
            SmartDashboard.putNumber("Yaw", photonTrackedTarget.getYaw());
            SmartDashboard.putNumber("ID", photonTrackedTarget.getFiducialId());
            SmartDashboard.putNumber("Pitch", photonTrackedTarget.getPitch());
        } 
    }
 }