package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TheGreatBalancingAct;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class DirectBalance extends SequentialCommandGroup {
    public DirectBalance(SwerveAutoBuilder autoBuilder, SwerveDrivetrain swerve, Shooter shoot, Wrist wrist) {
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("DirectBalance");
        
        addCommands(
            Commands.sequence(
                Commands.deadline(
                    new WaitCommand(14.5),
                    Commands.sequence(
                        Commands.runOnce(() -> swerve.getImu().zeroAll()),
                        autoBuilder.resetPose(pathGroup.get(0)),
                        wrist.motionMagicCommand(WristConstants.kWristHigh)),
                        shoot.outtakeHigh(),
                        Commands.waitSeconds(1),
                        shoot.setPower(0),
                        wrist.motionMagicCommand(WristConstants.kWristStow),
                        autoBuilder.followPathWithEvents(pathGroup.get(0)),
                        new TheGreatBalancingAct(swerve)
                    ),
                    Commands.runOnce(() -> swerve.towModules()),
                    Commands.runOnce(() -> swerve.getImu().setOffset(180)
                )
            )
        );
    }
    
}
