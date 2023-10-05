package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class TwoPiece extends SequentialCommandGroup {
    public TwoPiece(SwerveAutoBuilder autoBuilder, SwerveDrivetrain swerve, Shooter shoot, Wrist wrist) {
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("TwoPiece");
        
        addCommands(
            Commands.sequence(
                autoBuilder.resetPose(pathGroup.get(0)),
                autoBuilder.followPathWithEvents(pathGroup.get(0))
                ,
                autoBuilder.followPathWithEvents(pathGroup.get(1))
                ));
    }
}
