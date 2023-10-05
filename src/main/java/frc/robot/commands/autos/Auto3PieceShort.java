package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.pvpe.DrivetrainPoseEstm;
import frc.robot.subsystems.vision.pvpe.FollowTrajectoryCommand;


public class Auto3PieceShort extends SequentialCommandGroup {
    public Auto3PieceShort(SwerveAutoBuilder autoBuilder, SwerveDrivetrain swerve, Shooter shoot, Wrist wrist) {
        //List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("3Piece Short");
        DrivetrainPoseEstm poseEstm = new DrivetrainPoseEstm(swerve);

        TrajectoryConfig config = new TrajectoryConfig(1.25 * 0.75, 2.5 / 2);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(1.75, 4.43, new Rotation2d(0)), 
        List.of(
            new Translation2d(2.5, 4.43),
            new Translation2d(3, 4.43)
        ), 
        new Pose2d(3.75, 4.43, new Rotation2d(0)), config);

        FollowTrajectoryCommand trajFollower = new FollowTrajectoryCommand(trajectory, poseEstm);
        
        addCommands(
            Commands.sequence(
                Commands.runOnce(() -> swerve.getImu().zeroAll()),
                // wrist.motionMagicCommand(WristConstants.kWristStow),
                // shoot.outtakeHigh(),
                trajFollower
                

                // Go to intake
                // Commands.parallel(
                //     wrist.motionMagicCommand(WristConstants.kWristLowPickup),
                //     autoBuilder.followPathWithEvents(pathGroup.get(0)),
                //     shoot.setPower(ShooterConstants.kTopIntakePower.get(), ShooterConstants.kBottomIntakePower.get())
                // ),

                // // Intake and come back to shoot
                // Commands.sequence(
                //     Commands.runOnce(() -> swerve.towModules()),
                //     Commands.waitSeconds(0.5),
                //     wrist.motionMagicCommand(WristConstants.kWristStow),
                //     Commands.parallel(
                //         autoBuilder.followPathWithEvents(pathGroup.get(1))
                //     ),
                //     shoot.outtakeLow()
                // ),

                // // Go back to intake
                // Commands.parallel(
                //     autoBuilder.followPathWithEvents(pathGroup.get(2)),
                //     Commands.sequence(
                //         Commands.waitSeconds(0.5),
                //         wrist.motionMagicCommand(WristConstants.kWristLowPickup)
                //     ),
                //     shoot.setPower(ShooterConstants.kTopIntakePower.get(), ShooterConstants.kBottomIntakePower.get())
                // ),
                
                // // Intake and come back to shoot
                // ,

                // End auto, prepare imu for teleop
                // Commands.runOnce(() -> swerve.getImu().setOffset(180))

                // ,
                // shoot.setPower(ShooterConstants.kTopIntakeNeutralPower.get(), ShooterConstants.kBottomIntakeNeutralPower.get()),
                // autoBuilder.followPathWithEvents(pathGroup.get(3)),
                // // wrist.motionMagicCommand(WristConstants.kWristLow),
                // shoot.outtakeLow()
                // ,
                // Commands.parallel(
                //     wrist.motionMagicCommand(WristConstants.kWristStow),
                //     autoBuilder.followPathWithEvents(pathGroup.get(4))
                // )
                //,
                // Commands.parallel(
                //     // wrist.motionMagicCommand(WristConstants.kWristLowPickup),
                //     autoBuilder.followPathWithEvents(pathGroup.get(5)),
                //     shoot.setPower(ShooterConstants.kTopIntakePower.get(), ShooterConstants.kBottomIntakePower.get())
                // ),
                // Commands.parallel(
                //     wrist.motionMagicCommand(WristConstants.kWristStow),
                //     autoBuilder.followPathWithEvents(pathGroup.get(6)),
                //     shoot.setPower(ShooterConstants.kTopIntakeNeutralPower.get(), ShooterConstants.kBottomIntakeNeutralPower.get())
                // )
                // ,
                // shoot.setPower(ShooterConstants.kTopIntakeNeutralPower.get(), ShooterConstants.kBottomIntakeNeutralPower.get()),
                // autoBuilder.followPathWithEvents(pathGroup.get(7)),
                // wrist.motionMagicCommand(WristConstants.kWristLow),
                // shoot.outtakeLow(),
                // wrist.motionMagicCommand(WristConstants.kWristStow)
            )
        );
    }
    
}
