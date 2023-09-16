package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TheGreatBalancingAct;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class Auto3PieceLong extends SequentialCommandGroup {
    public Auto3PieceLong(SwerveAutoBuilder autoBuilder,  SwerveDrivetrain swerve, Shooter shoot, Wrist wrist) {
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("3Piece Long");
        
        addCommands(
            Commands.sequence(
                autoBuilder.resetPose(pathGroup.get(0)),
                wrist.motionMagicCommand(WristConstants.kWristHigh)),
                shoot.outtakeHigh(),
                wrist.motionMagicCommand(WristConstants.kWristStow),
                autoBuilder.followPathWithEvents(pathGroup.get(0)),
                autoBuilder.followPathWithEvents(pathGroup.get(1)),
                autoBuilder.followPathWithEvents(pathGroup.get(2)),
                autoBuilder.followPathWithEvents(pathGroup.get(3)),
                wrist.motionMagicCommand(WristConstants.kWristGround),
                shoot.intake(),
                Commands.parallel(
                    wrist.motionMagicCommand(WristConstants.kWristStow),
                    shoot.setPower(ShooterConstants.kTopIntakeNeutralPower.get(), ShooterConstants.kBottomIntakeNeutralPower.get()),
                    autoBuilder.followPathWithEvents(pathGroup.get(4)),
                    autoBuilder.followPathWithEvents(pathGroup.get(5)),
                    autoBuilder.followPathWithEvents(pathGroup.get(6))
                ),
                wrist.motionMagicCommand(WristConstants.kWristHigh),
                shoot.outtakeHigh(),
                wrist.motionMagicCommand(WristConstants.kWristStow),
                autoBuilder.followPathWithEvents(pathGroup.get(7)),
                autoBuilder.followPathWithEvents(pathGroup.get(8)),
                autoBuilder.followPathWithEvents(pathGroup.get(9)),
                wrist.motionMagicCommand(WristConstants.kWristGround),
                shoot.intake(),
                Commands.parallel(
                    wrist.motionMagicCommand(WristConstants.kWristStow),
                    shoot.setPower(ShooterConstants.kTopIntakeNeutralPower.get(), ShooterConstants.kBottomIntakeNeutralPower.get()),
                    autoBuilder.followPathWithEvents(pathGroup.get(10)),
                    autoBuilder.followPathWithEvents(pathGroup.get(11)),
                    autoBuilder.followPathWithEvents(pathGroup.get(12))
                ),
                wrist.motionMagicCommand(WristConstants.kWristHigh),
                shoot.outtakeHigh(),
                wrist.motionMagicCommand(WristConstants.kWristStow)
            );
    }
    
}
