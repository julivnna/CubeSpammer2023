package frc.robot.commands.autos.ramsete;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class RamseteAutoBuilder extends BaseAutoBuilder {
    private final RamseteController controller;
    private final Consumer<ChassisSpeeds> outputChassisSpeeds;
    private final Subsystem[] driveRequirements;

    /**
     * Create an auto builder that will create command groups that will handle path following and
     * triggering events.
     *
     * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
     *
     * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
     *     to provide this.
     * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
     *     be called once at the beginning of an auto.
     * @param translationConstants PID Constants for the controller that will correct for translation
     *     error
     * @param rotationConstants PID Constants for the controller that will correct for rotation error
     * @param outputChassisSpeeds A function that takes the output ChassisSpeeds from path following
     *     commands
     * @param eventMap Map of event marker names to the commands that should run when reaching that
     *     marker.
     * @param driveRequirements The subsystems that the path following commands should require.
     *     Usually just a Drive subsystem.
     */
    public RamseteAutoBuilder(
        Supplier<Pose2d> poseSupplier,
        Consumer<Pose2d> resetPose,
        RamseteController controller,
        Consumer<ChassisSpeeds> outputChassisSpeeds,
        Map<String, Command> eventMap,
        Subsystem... driveRequirements) {
        this(
            poseSupplier,
            resetPose,
            controller,
            outputChassisSpeeds,
            eventMap,
            false,
            driveRequirements);
    }


    /**
     * Create an auto builder that will create command groups that will handle path following and
     * triggering events.
     *
     * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
     *
     * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
     *     to provide this.
     * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
     *     be called once at the beginning of an auto.
     * @param outputChassisSpeeds A function that takes the output ChassisSpeeds from path following
     *     commands
     * @param eventMap Map of event marker names to the commands that should run when reaching that
     *     marker.
     * @param useAllianceColor Should the path states be automatically transformed based on alliance
     *     color? In order for this to work properly, you MUST create your path on the blue side of
     *     the field.
     * @param driveRequirements The subsystems that the path following commands should require.
     *     Usually just a Drive subsystem.
     */
    public RamseteAutoBuilder(
        Supplier<Pose2d> poseSupplier,
        Consumer<Pose2d> resetPose,
        RamseteController controller,
        Consumer<ChassisSpeeds> outputChassisSpeeds,
        Map<String, Command> eventMap,
        boolean useAllianceColor,
        Subsystem... driveRequirements) {
        super(poseSupplier, resetPose, eventMap, DrivetrainType.HOLONOMIC, useAllianceColor);

        this.controller = controller;
        this.outputChassisSpeeds = outputChassisSpeeds;
        this.driveRequirements = driveRequirements;
    }

    public CommandBase followPath(Trajectory trajectory) {
        return new RamseteAutoCommand(
            trajectory,
            poseSupplier,
            resetPose,
            controller,
            outputChassisSpeeds,
            driveRequirements);
    }

    @Override
    public CommandBase followPath(PathPlannerTrajectory trajectory) {
        return new RamseteAutoCommand(
            trajectory,
            poseSupplier,
            resetPose,
            controller,
            outputChassisSpeeds,
            useAllianceColor,
            driveRequirements);
    }
}
