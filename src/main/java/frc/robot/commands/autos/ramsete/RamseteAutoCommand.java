package frc.robot.commands.autos.ramsete;

import edu.wpi.first.wpilibj2.command.CommandBase;
    
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;

public class RamseteAutoCommand extends CommandBase {
    private RamseteSwerveController controller;
    private Trajectory trajectory;
    private Supplier<Pose2d> poseSupplier;
    private Consumer<Pose2d> resetPose;
    private Consumer<ChassisSpeeds> outputChassisSpeeds;

    private double trajectoryDuration;

    private Trajectory.State currentTrajectoryState;

    public RamseteAutoCommand(
            PathPlannerTrajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            Consumer<Pose2d> resetPose,
            RamseteController controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            boolean useAllianceColor,
            Subsystem... requirements) {

        if (useAllianceColor) {
            this.trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());
        } else {
            this.trajectory = trajectory;
        }
        this.poseSupplier = poseSupplier;
        this.resetPose = resetPose;
        this.outputChassisSpeeds = outputChassisSpeeds;
        this.controller = new RamseteSwerveController(trajectory, controller);

        addRequirements(requirements);
    }

    public RamseteAutoCommand(
            Trajectory trajectory,
            Supplier<Pose2d> poseSupplier,
            Consumer<Pose2d> resetPose,
            RamseteController controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            Subsystem... requirements) {
        this.trajectory = trajectory;
        this.poseSupplier = poseSupplier;
        this.resetPose = resetPose;
        this.outputChassisSpeeds = outputChassisSpeeds;
        this.controller = new RamseteSwerveController(trajectory, controller);

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        //m_poseEstimator.resetPosition(null, getModulePositions(), getCtrlsPoseEstimate());
        resetPose.accept(controller.getInitialPose());
        currentTrajectoryState = trajectory.sample(0);
        controller.startPath();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = controller.calculateSpeeds(poseSupplier.get());
        //m_swerve.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
        outputChassisSpeeds.accept(speeds);

        currentTrajectoryState = controller.getCurrentState(); // to be verified: use autoCtrl state or use drive pose????
        //trajectory.sample(poseSupplier.get());???????????????
    }

    private static final double kTolerance = 0.1; // Tolerance for position-based check

    @Override
    public boolean isFinished() {
        // Check if the trajectory has finished or arrived based on position
        if (currentTrajectoryState == null) {
            return true;
        }

        // to be done: better to add it to RamseteSwerveController
        double currentDistance = currentTrajectoryState.poseMeters.getTranslation().getNorm();
        double remainingDistance = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getTranslation().getNorm() - currentDistance;

        if (remainingDistance <= kTolerance) {
            //System.out.println("Trajectory Arrived");
            // Perform any actions needed when the trajectory arrives
            return true;
        }

        // to be added: add the duration check to finish the route, better to add it to RamseteSwerveController
        
        return false; // You may implement a condition to end the command when the trajectory is complete
    }


}