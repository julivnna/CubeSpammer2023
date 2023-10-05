package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class EasyTesting extends SequentialCommandGroup {
    private static List<Trajectory> trajectories = new ArrayList<Trajectory>();
    private static List<SwerveControllerCommand> trajectoryCommands = new ArrayList<SwerveControllerCommand>();

    public EasyTesting(boolean testing, SwerveDrivetrain swerve, Shooter shooter, Wrist wrist) {
        addTrajectory(3, swerve);
        
        addCommands(
            Commands.sequence(
                Commands.runOnce(() -> swerve.getImu().zeroAll()),
                Commands.runOnce(() -> swerve.resetOdometry(trajectories.get(0).getInitialPose())),

                //preload
                wrist.motionMagicCommand(WristConstants.kWristStow),
                shooter.outtakeHigh(),

                //go to first cube and intake
                Commands.parallel(
                    wrist.motionMagicCommand(WristConstants.kWristLowPickup),
                    trajectoryCommands.get(0),
                    shooter.setPower(ShooterConstants.kTopIntakePower.get(), ShooterConstants.kBottomIntakePower.get())
                ),

                Commands.runOnce(() -> swerve.towModules()),

                //go to score 2nd piece
                Commands.waitSeconds(0.5),
                wrist.motionMagicCommand(WristConstants.kWristStow),
                trajectoryCommands.get(1),
                shooter.outtakeLow(),

                Commands.runOnce(() -> swerve.towModules()),

                //go to second cube and intake
                Commands.parallel(
                    trajectoryCommands.get(2),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        wrist.motionMagicCommand(WristConstants.kWristLowPickup)
                    ),
                    shooter.setPower(ShooterConstants.kTopIntakePower.get(), ShooterConstants.kBottomIntakePower.get())
                ),

                Commands.runOnce(() -> swerve.towModules()),

                //got to score 3rd piece
                Commands.waitSeconds(0.5),
                Commands.parallel(
                    wrist.motionMagicCommand(WristConstants.kWristStow),
                    trajectoryCommands.get(3)
                ),
                shooter.outtakeHigh(),

                Commands.runOnce(() -> swerve.towModules()),

                //leave community
                Commands.waitSeconds(0.5),
                trajectoryCommands.get(4),

                Commands.runOnce(() -> swerve.towModules())
            )
        );
    }

    private static void addTrajectory(int waypoints, SwerveDrivetrain swerve) {
        TrajectoryConfig config = new TrajectoryConfig(PathPlannerConstants.kPPMaxVelocity, PathPlannerConstants.kPPMaxAcceleration);
        PIDController translation = new PIDController(PathPlannerConstants.kPPTranslationPIDConstants.kP, PathPlannerConstants.kPPTranslationPIDConstants.kI, PathPlannerConstants.kPPTranslationPIDConstants.kD);
        Constraints constraints = new Constraints(PathPlannerConstants.kPPPathConstraints.maxVelocity, PathPlannerConstants.kPPPathConstraints.maxAcceleration);
        ProfiledPIDController rotation = new ProfiledPIDController(PathPlannerConstants.kPPRotationPIDConstants.kP, PathPlannerConstants.kPPRotationPIDConstants.kI, PathPlannerConstants.kPPRotationPIDConstants.kD, constraints);

        List<Pose2d> pose2ds = new ArrayList<Pose2d>();
        for (int i = 1; i <= waypoints; i++) {
            SmartDashboard.putNumber("traj" + (trajectoryCommands.size() + 1) + "X" + i, 0);
            SmartDashboard.putNumber("traj" + (trajectoryCommands.size() + 1) + "Y" + i, 0);
            SmartDashboard.putNumber("traj" + (trajectoryCommands.size() + 1) + "R" + i, 0);

            pose2ds.add(new Pose2d(
                SmartDashboard.getNumber("traj" + (trajectoryCommands.size() + 1) + "X" + i, 0),
                SmartDashboard.getNumber("traj" + (trajectoryCommands.size() + 1) + "Y" + i, 0),
                new Rotation2d(SmartDashboard.getNumber("traj" + (trajectoryCommands.size() + 1) + "R" + i, 0))
            ));
        }
        
        Trajectory temp = TrajectoryGenerator.generateTrajectory(pose2ds, config);
        trajectories.add(temp);
        trajectoryCommands.add(new SwerveControllerCommand(temp, swerve::getPose, SwerveDriveConstants.kDriveKinematics, translation, translation, rotation, swerve::setModuleStates, swerve));
    }
}
