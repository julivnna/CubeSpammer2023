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


public class Auto3PieceShort extends SequentialCommandGroup {
    public Auto3PieceShort(SwerveAutoBuilder autoBuilder, SwerveDrivetrain swerve, Shooter shoot, Wrist wrist) {
        List<PathPlannerTrajectory> pathGroup = PathPlannerAutos.getPathGroup("3Piece Short");
        
        addCommands(
            Commands.sequence(
                Commands.runOnce(() -> swerve.getImu().zeroAll()),
                autoBuilder.resetPose(pathGroup.get(0)),
                wrist.motionMagicCommand(WristConstants.kWristStow),
                shoot.outtakeHigh(),

                // Go to intake
                Commands.parallel(
                    wrist.motionMagicCommand(WristConstants.kWristLowPickup),
                    autoBuilder.followPathWithEvents(pathGroup.get(0)),
                    shoot.setPower(ShooterConstants.kTopIntakePower.get(), ShooterConstants.kBottomIntakePower.get())
                ),

                // Intake and come back to shoot
                Commands.sequence(
                    Commands.runOnce(() -> swerve.towModules()),
                    Commands.waitSeconds(0.5),
                    wrist.motionMagicCommand(WristConstants.kWristStow),
                    Commands.parallel(
                        autoBuilder.followPathWithEvents(pathGroup.get(1))
                    ),
                    shoot.outtakeLow()
                ),

                // Go back to intake
                Commands.parallel(
                    autoBuilder.followPathWithEvents(pathGroup.get(2)),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        wrist.motionMagicCommand(WristConstants.kWristLowPickup)
                    ),
                    shoot.setPower(ShooterConstants.kTopIntakePower.get(), ShooterConstants.kBottomIntakePower.get())
                ),
                
                // Intake and come back to shoot
                Commands.sequence(
                    Commands.runOnce(() -> swerve.towModules()),
                    Commands.waitSeconds(0.5),
                    wrist.motionMagicCommand(WristConstants.kWristStow),
                    shoot.setPower(ShooterConstants.kTopIntakeNeutralPower.get(), ShooterConstants.kBottomIntakeNeutralPower.get()),
                    autoBuilder.followPathWithEvents(pathGroup.get(3))
                ),
                shoot.outtakeLow(),

                // End auto, prepare imu for teleop
                Commands.runOnce(() -> swerve.getImu().setOffset(180))

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
