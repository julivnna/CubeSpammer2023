package frc.robot.subsystems.vision.pvpe;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.SwerveDriveConstants.CANCoderConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swerve.CANSwerveModule;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveModule;
import static frc.robot.Constants.SwerveDriveConstants.*;

public class FollowTrajectoryCommand extends CommandBase {
    private DrivetrainPoseEstm autoDrive;
    private AutoController autoCtrl;
    private Trajectory trajectory;

    private double trajectoryDuration;

    private Trajectory.State currentTrajectoryState;

    public FollowTrajectoryCommand(Trajectory trajectory, DrivetrainPoseEstm autoDrive) {
        //requires(drive);
        this.trajectory = trajectory;
        trajectoryDuration = trajectory.getTotalTimeSeconds(); // to be checed if it's correct...
        this.autoDrive = autoDrive;
        this.autoCtrl = new AutoController(trajectory);
        SmartDashboard.putNumber("trajectoryDuration", trajectoryDuration);
      
    }

    @Override
    public void initialize() {
        //m_poseEstimator.resetPosition(null, getModulePositions(), getCtrlsPoseEstimate());
        autoDrive.resetAutoDriveOdometry(autoCtrl.getInitialPose().getRotation(), autoCtrl.getInitialPose() );
        currentTrajectoryState = trajectory.sample(0);
        
    }

    private boolean started = false;
    @Override
    public void execute() {

        if(!started)
        {
            started = true;
            autoCtrl.startPath();
        }
        //desiredDtState = trajectory.sample(System.currentTimeMillis() / 1000.0); // Replace with your time source
        //Pose2d robotPose = odometry.getPoseMeters();
        //ChassisSpeeds adjustedSpeeds = ramseteController.calculate(robotPose, desiredDtState);
        
        //DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
        //drive.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
        //odometry.update(desiredState.poseMeters.getRotation(), drive.getLeftEncoderDistance(), drive.getRightEncoderDistance());


        ChassisSpeeds speeds = autoCtrl.getCurMotorCmds(autoDrive.getCtrlsPoseEstimate());
        //m_swerve.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
        autoDrive.autoDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
        autoDrive.updateOdometry();

        currentTrajectoryState = autoCtrl.getCurrentState(); // to be verified: use autoCtrl state or use odometry pose????
        //trajectory.sample(drive.getPose());???????????????
        
        
    }

    private static final double kTolerance = 0.1; // Tolerance for position-based check

    @Override
    public boolean isFinished() {
        // Check if the trajectory has finished or arrived based on position
        if (currentTrajectoryState == null) {
            SmartDashboard.putBoolean("trajectory isFinished by null", true);
            return true;
        }

        // to be done: better to add it to AutoController
        double currentDistance = currentTrajectoryState.poseMeters.getTranslation().getNorm();
        double remainingDistance = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getTranslation().getNorm() - currentDistance;

        if (remainingDistance <= kTolerance) {
            //System.out.println("Trajectory Arrived");
            // Perform any actions needed when the trajectory arrives
            SmartDashboard.putBoolean("trajectory isFinished by distance", true);
            return true;
        }

        // to be added: add the duration check to finish the route, better to add it to AutoController
        SmartDashboard.putBoolean("trajectory isFinished", false);
        return false; // You may implement a condition to end the command when the trajectory is complete
    }


}

