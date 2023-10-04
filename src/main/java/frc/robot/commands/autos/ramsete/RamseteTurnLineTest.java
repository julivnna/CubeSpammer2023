package frc.robot.commands.autos.ramsete;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.PathPlannerAutos;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class RamseteTurnLineTest extends SequentialCommandGroup {
    public RamseteTurnLineTest(RamseteAutoBuilder autoBuilder, SwerveDrivetrain swerve) {        
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("TurnLineTest");

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.getImu().setOffset(180)),
            autoBuilder.resetPose(pathGroup.get(0)),
            autoBuilder.followPath(pathGroup.get(0))
        );
    }
    
}
