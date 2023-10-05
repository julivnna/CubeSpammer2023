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
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class EasyTesting extends SequentialCommandGroup {
    private static List<SwerveControllerCommand> trajectoryCommands = new ArrayList<SwerveControllerCommand>();

    public EasyTesting(boolean testing, SwerveDrivetrain swerve, Shooter shooter, Wrist wrist) {
        Trajectory initToCube1, cube1ToScore, scoreToCube2, cube2ToScore2, score2ToOutside;

        addTrajectory(3, swerve);
        
        addCommands(
            Commands.sequence(
                trajectoryCommands.get(0)
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
        trajectoryCommands.add(new SwerveControllerCommand(temp, swerve::getPose, SwerveDriveConstants.kDriveKinematics, translation, translation, rotation, swerve::setModuleStates, swerve));
    }
}
