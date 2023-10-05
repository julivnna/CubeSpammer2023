// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.SwerveDriveConstants.CANCoderConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.swerve.CANSwerveModule;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Limelight.LightMode;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelperUser;

import static frc.robot.Constants.SwerveDriveConstants.*;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainPoseEstm {
  //public static final double kMaxSpeed = 3.0; // 3 meters per second
  //public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final PoseEstimator m_poseEstimator;
  private final Gyro m_gyro;
  //public PhotonCameraWrapper pcw;
  private Limelight limelight = new Limelight(VisionConstants.kLimelightFrontName);
  private LimelightHelperUser limelightUser = new LimelightHelperUser(VisionConstants.kLimelightFrontName);
  private SwerveDrivetrain drive;

  public DrivetrainPoseEstm(SwerveDrivetrain drive) {
    if(limelight != null) {
      limelight.setLightState(LightMode.OFF);
      limelight.setPipeline(4);
    }
    //pcw = new PhotonCameraWrapper();
    this.drive = drive;
    m_gyro = drive.getImu();
    m_gyro.zeroAll(); //m_gyro.reset(); //to be checked: if it's face to 0?
    
    // to be added: reset encoder?

    m_poseEstimator =
      new PoseEstimator(
          m_kinematics,
          m_gyro.getRotation2d(), // to be confirmed: it is 0?
          drive.getModulePositions(),
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void autoDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    /*SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);*/
    drive.setModuleStates(swerveModuleStates);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.updateWithTime(
      Timer.getFPGATimestamp(),
        m_gyro.getRotation2d(),
        drive.getModulePositions());

    // Also apply vision measurements. We use 0.3 seconds in the past as an example -- on
    // a real robot, this must be calculated based either on latency or timestamps.

    // m_poseEstimator.addVisionMeasurement(
    //     ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
    //         m_poseEstimator.getEstimatedPosition()),
    //     Timer.getFPGATimestamp() - 0.3);
    // Optional<EstimatedRobotPose> result =
    //             pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

    // if (result.isPresent()) {
    //     EstimatedRobotPose camPose = result.get();
    //     m_poseEstimator.addVisionMeasurement(
    //             camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    if(limelight.hasValidTarget()) {
      m_poseEstimator.addVisionMeasurement(limelightUser.getPose3d().toPose2d(), Timer.getFPGATimestamp() - 0.0);
      SmartDashboard.putNumber("getcamPose_X", limelightUser.getPose3d().toPose2d().getX());
      SmartDashboard.putNumber("getcamPose_Y", limelightUser.getPose3d().toPose2d().getY());
    }
        
    //     SmartDashboard.putNumber("getcamPose_X", camPose.estimatedPose.toPose2d().getX());
    //     SmartDashboard.putNumber("getcamPose_Y", camPose.estimatedPose.toPose2d().getY());
    // }
    
    //SmartDashboard.putBoolean("getApriltagPresent", result.isPresent());

    SmartDashboard.putNumber("getEstimatedPosition_X", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("getEstimatedPosition_Y", m_poseEstimator.getEstimatedPosition().getY());
  }

  /**
     * Force the pose estimator to a particular pose. This is useful for indicating to the software
     * when you have manually moved your robot in a particular position on the field (EX: when you
     * place it on the field at the start of the match).
 * @param swerveModulePositions
 * @param rotation2d
     */
  public void resetAutoDriveOdometry(Rotation2d rotation2d, Pose2d pose) {
    // m_frontLeft.resetEncoder();
    // m_frontRight.resetEncoder();
    // m_backLeft.resetEncoder();
    // m_backRight.resetEncoder();
    drive.resetEncoders();
    m_poseEstimator.resetPosition(rotation2d, drive.getModulePositions(), pose);
  }

  /**
   * @return The current best-guess at drivetrain position on the field.
   */
  public Pose2d getCtrlsPoseEstimate() { //getPoseEst
      return m_poseEstimator.getEstimatedPosition();
  }

  public SwerveDrivetrain getDrivetrain()
  {
    return drive;
  }
}
