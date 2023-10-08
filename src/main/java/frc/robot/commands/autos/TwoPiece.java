package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class TwoPiece extends SequentialCommandGroup {
    public TwoPiece(SwerveAutoBuilder autoBuilder, SwerveDrivetrain swerve, Shooter shoot, Wrist wrist) {
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("TwoPiece");
        
        addCommands(
            Commands.sequence(
                autoBuilder.resetPose(pathGroup.get(0)),
                shoot.outtakeAutoHigh(),
                
                
                
                // Go to position to intake
                autoBuilder.followPathWithEvents(pathGroup.get(0)),
                
                // Intake and drive forward
                Commands.parallel(
                    wrist.motionMagicCommand(WristConstants.kWristLowPickup),
                    autoBuilder.followPathWithEvents(pathGroup.get(1)),
                    shoot.setPower(ShooterConstants.kTopIntakePower.get(), ShooterConstants.kBottomIntakePower.get())
                ),

                wrist.motionMagicCommand(WristConstants.kWristStow),
                shoot.setPower(ShooterConstants.kTopIntakeNeutralPower.get(), ShooterConstants.kBottomIntakeNeutralPower.get()),
                Commands.runOnce(() -> swerve.getImu().setOffset(0))
                
                // Stow and go to grid
                // Commands.sequence(
                //     // Commands.waitSeconds(0.5),
                //     wrist.motionMagicCommand(WristConstants.kWristStow),
                //     autoBuilder.followPathWithEvents(pathGroup.get(2))
                // )
                ));
    }
}