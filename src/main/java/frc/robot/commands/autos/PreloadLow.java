package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class PreloadLow extends SequentialCommandGroup {
    public PreloadLow(SwerveDrivetrain swerve, Shooter shoot, Wrist wrist) {
        addCommands(
            Commands.runOnce(() -> swerve.getImu().zeroAll()),
            wrist.motionMagicCommand(WristConstants.kWristStow),
            Commands.waitSeconds(0.25),
            shoot.outtakeLow()
        );
    }
}
