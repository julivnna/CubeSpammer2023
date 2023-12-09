package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class TestStraightOtherDirection extends SequentialCommandGroup {
    public TestStraightOtherDirection(SwerveAutoBuilder autoBuilder) {
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("TestStraightOtherDirection");
        
        addCommands(
            Commands.sequence(
                autoBuilder.resetPose(pathGroup.get(0)),
                autoBuilder.followPathWithEvents(pathGroup.get(0))));
    }
    
}
