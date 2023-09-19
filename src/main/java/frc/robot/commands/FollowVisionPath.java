package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autos.PathPlannerAutos;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.primalWallnut.PrimalSunflower;

public class FollowVisionPath extends SequentialCommandGroup {
    /**
     * Construct a new FollowVisionPath command
     * 
     * Uses primal sunflower to follow a path using pathplanner trajectory
     * 
     * @param autoBuilder Swerve Auto Builder for Path Planner
     * @param sunflower   Primal Sunflower
     */
    public FollowVisionPath(SwerveAutoBuilder autoBuilder, PrimalSunflower sunflower) {
        autoBuilder.followPathWithEvents(sunflower.usePlantFood());
    }
}