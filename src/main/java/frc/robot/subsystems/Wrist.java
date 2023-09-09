package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.filters.ExponentialSmoothingFilter;
import frc.robot.util.NerdyMath;

public class Wrist extends SubsystemBase implements Reportable {
    private TalonFX wrist;
    private int targetTicks = WristConstants.kWristStow;
    public BooleanSupplier atTargetPosition;
    private TalonSRX leftEncoder;
    private ExponentialSmoothingFilter joystickFilter = new ExponentialSmoothingFilter(WristConstants.kLowPassAlpha);

    public Wrist() {
        wrist = new TalonFX(WristConstants.kWristID);
        leftEncoder = new TalonSRX(WristConstants.kLeftEncoderID);
    }

    @Override
    public void periodic() {
        moveWristMotionMagic();
    }

    public void init() {
        wrist.setNeutralMode(NeutralMode.Brake);
        wrist.setInverted(false);
        leftEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 1000);
        leftEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.PulseWidthEncodedPosition, 1, 1000);
        //test
        wrist.config_kP(0, WristConstants.kWristP);
        wrist.config_kI(0, WristConstants.kWristI);
        wrist.config_kD(0, WristConstants.kWristD);
        wrist.config_kF(0, WristConstants.kWristF);

        wrist.configMotionCruiseVelocity(WristConstants.kWristCruiseVelocity);
        wrist.configMotionAcceleration(WristConstants.kWristMotionAcceleration);

    }

    public void resetEncoders(){
        double absoluteTicks = leftEncoder.getSelectedSensorPosition(0);
        wrist.setSelectedSensorPosition(absoluteTicks * WristConstants.kFalconTicksPerAbsoluteTicks, 0, 100);
    }

    public void moveWristJoystick(double currentJoystickOutput) {
        if (currentJoystickOutput > WristConstants.kWristDeadband) {
            if (wrist.getStatorCurrent() >= 45)
            {
                wrist.set(ControlMode.PercentOutput, 0);
            } else {
                wrist.set(ControlMode.PercentOutput, 0.3);
            }
        }

        else if (currentJoystickOutput < -WristConstants.kWristDeadband){
            if (wrist.getStatorCurrent() >= 45) {
                wrist.set(ControlMode.PercentOutput, 0);
            }
            else {
                wrist.set(ControlMode.PercentOutput, -0.3);
            }}

        else {
            wrist.set(ControlMode.PercentOutput, 0);
            wrist.setNeutralMode(NeutralMode.Brake);
        }
    }

    public void moveWristMotionMagicJoystick(double joystickInput, double percentExtended) {
        if (joystickInput < -0.1 || joystickInput > 0.1) {
            int tickChange = (int) (WristConstants.kJoystickScale * joystickInput);
            int currentTicks = (int) wrist.getSelectedSensorPosition();

            tickChange = (int) joystickFilter.calculate(tickChange);

            targetTicks = currentTicks + tickChange;
            targetTicks = (int) NerdyMath.clamp(targetTicks, WristConstants.kWristLowerLimit, WristConstants.kWristUpperLimit);
        }
        else {
            joystickFilter.calculate(0);
        }

        setTargetTicks(targetTicks);
    }


    public double getWristAngle() {
        double ticks = wrist.getSelectedSensorPosition(0);
        double angle = ticks * WristConstants.kDegreesPerTick %360;
        return angle;
    }

    public double getWristAngleRadians() {
        return Math.toRadians(getWristAngle());
    }

    public void moveWristMotionMagic() {
        double ff = WristConstants.kWristFF * Math.cos(getWristAngleRadians());
        wrist.set(ControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, ff);
    }

    public CommandBase motionMagicCommand(int position) {
        return Commands.runOnce(() -> {
            wrist.configMotionCruiseVelocity(WristConstants.kWristCruiseVelocity);
            wrist.configMotionAcceleration(WristConstants.kWristMotionAcceleration);
            setTargetTicks(position);
        });
    }

    public void setTargetTicks(int targetTicks) {
        this.targetTicks = targetTicks;
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
                SmartDashboard.putNumber("Wrist Motor Output", wrist.getMotorOutputPercent());
                SmartDashboard.putNumber("Wrist Angle", Math.toDegrees(getWristAngle()));
                SmartDashboard.putNumber("Wrist Velocity", wrist.getSelectedSensorVelocity());
            case MEDIUM:
                SmartDashboard.putNumber("Wrist Current", wrist.getStatorCurrent());
                SmartDashboard.putNumber("Wrist Voltage", wrist.getMotorOutputVoltage());
            case MINIMAL:
                SmartDashboard.putNumber("Wrist Ticks", wrist.getSelectedSensorPosition());
                SmartDashboard.putNumber("Target Wrist Ticks", targetTicks);
                break;
        }
        
    }

    @Override
    public void initShuffleboard(LOG_LEVEL level) { 
        if (level == LOG_LEVEL.OFF || level == LOG_LEVEL.MINIMAL) {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Wrist");
        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addNumber("Motor Output", wrist::getMotorOutputPercent);
                tab.addString("Control Mode", wrist.getControlMode()::toString);
                // tab.addNumber("Wrist Target Velocity", wrist::getActiveTrajectoryVelocity); 
                // tab.addNumber("Closed loop error", wrist::getClosedLoopError);

            case MEDIUM:
                tab.addNumber("Wrist Current", wrist::getStatorCurrent);
                tab.addNumber("Wrist Velocity", wrist::getSelectedSensorVelocity);
                tab.addNumber("Wrist Voltage", wrist::getMotorOutputVoltage);
                tab.addNumber("Wrist Percent Output", wrist::getMotorOutputPercent);

            case MINIMAL:
                tab.addNumber("Current Wrist Ticks", wrist::getSelectedSensorPosition);
                tab.addNumber("Target Wrist Ticks", () -> targetTicks);
                tab.addBoolean("At target position", atTargetPosition);
                break;
        }
        }
        
    }

