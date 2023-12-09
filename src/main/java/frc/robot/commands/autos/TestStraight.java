package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class TestStraight extends SequentialCommandGroup {
    public TestStraight(SwerveAutoBuilder autoBuilder) {
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("TestStraight");
        
        addCommands(
            Commands.sequence(
                autoBuilder.resetPose(pathGroup.get(0)),
                autoBuilder.followPathWithEvents(pathGroup.get(0))));
    }
    
}
