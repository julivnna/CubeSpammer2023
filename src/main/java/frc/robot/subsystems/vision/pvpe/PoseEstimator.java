package frc.robot.subsystems.vision.pvpe;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;

public class PoseEstimator extends SwerveDrivePoseEstimator{

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your
    // various sensors. Smaller numbers will cause the filter to "trust" the
    // estimate from that particular
    // component more than the others. This in turn means the particualr component
    // will have a stronger
    // influence on the final pose estimate.
    Matrix<N5, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05);
    Matrix<N3, N1> localMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));
    Matrix<N3, N1> visionMeasurementStdDevs =
            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

    public PoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs) {
        super(kinematics, gyroAngle, modulePositions, initialPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        //TODO Auto-generated constructor stub
    }

    public PoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters) {
        super(kinematics, gyroAngle, modulePositions, initialPoseMeters);
        //TODO Auto-generated constructor stub
    }
    /*
    void	addVisionMeasurement​(Pose2d visionRobotPoseMeters, double timestampSeconds)

void	addVisionMeasurement​(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3,​N1> visionMeasurementStdDevs)	

Pose2d	getEstimatedPosition()	

void	resetPosition​(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters)	

void	setVisionMeasurementStdDevs​(Matrix<N3,​N1> visionMeasurementStdDevs)	

Pose2d	update​(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions)	

Pose2d	updateWithTime​(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) 
*/

  /** Represents a single vision pose with associated standard deviations. */
//   public static record VisionUpdate(Pose2d pose, Matrix<N3, N1> stdDevs) {
//     public static final Comparator<VisionUpdate> compareDescStdDev =
//         (VisionUpdate a, VisionUpdate b) -> {
//           return -Double.compare(
//               a.stdDevs().get(0, 0) + a.stdDevs().get(1, 0),
//               b.stdDevs().get(0, 0) + b.stdDevs().get(1, 0));
//         };
//   }

  /** Represents a single vision pose with a timestamp and associated standard deviations. */
//   public static record TimestampedVisionUpdate(
//       double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}
    
}
