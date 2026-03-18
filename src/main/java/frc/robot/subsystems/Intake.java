package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.UnpaidIntern;

public class Intake extends SubsystemBase {

    private final TalonFX motorA;
    private final TalonFX motorB;

    private double lastTime = 0.0;
    private final double interval = 0.3;

    private boolean targetIsUp = false;
    private boolean enabledScheduler = false;

    private static final double DEADBAND = 0;
    private static final double MAX_SPEED = 0.2;
    private static final double MIN_SPEED = -0.2;

    // private static final double INTAKE_SPEED = 1;

    private static final double ARM_DOWN_MOVE = -0.17;
    private static final double ARM_UP_MOVE = 0.20;
    private static final double ARM_HOLD_STRONG = 0.01;

    private static final double STALL_CURRENT = 40.0;

    private static final int CURRENT_SAMPLES = 12;
    private final double[] currentBuf = new double[CURRENT_SAMPLES];
    private int currentIdx = 0;

    private static final int REQUIRED_STALL_LOOPS = 5;
    private int stallLoops = 0;

    private boolean armStalled = false;

    public Intake(int intakeMotorCANId, int armMotorCANId, String canbus) {
        motorA = new TalonFX(intakeMotorCANId, canbus);
        motorB = new TalonFX(armMotorCANId, canbus);
        UnpaidIntern.zeroEncoder(motorA);
        UnpaidIntern.zeroEncoder(motorB);
    }

    public void setTargetState(boolean isUp) {
        if (isUp) {
            targetIsUp = true;
            UnpaidIntern.stop(motorA);
        } else {
            targetIsUp = false;
        }
    }

    public void setSchedulerStatus(boolean isEnabled) {
        if (isEnabled) {
            enabledScheduler = true;
        }
    }

    public void schedulerChangesCall() {
        if (enabledScheduler && targetIsUp) {
            timedArmUp();
        }
    }

    public void bringIntakeDown() {
        targetIsUp = false;
        timedArmDown();
    }

    public void setIntake(double speed) {
        if (speed > MAX_SPEED) speed = MAX_SPEED;
        if (speed < MIN_SPEED) speed = MIN_SPEED;
        RobotContainer.speedChange = 0.5;
        UnpaidIntern.setPercentWithDeadband(motorA, speed, DEADBAND);
    }

    private double filteredCurrent() {
        currentBuf[currentIdx] = motorB.getStatorCurrent().getValueAsDouble();
        currentIdx = (currentIdx + 1) % CURRENT_SAMPLES;
        double sum = 0;
        for (double v : currentBuf) sum += v;
        System.out.println("Current: "+(sum/CURRENT_SAMPLES));
        WebServer.putNumber("IntakeArmVoltage", (sum / CURRENT_SAMPLES));
        return sum / CURRENT_SAMPLES;
    }

    private boolean armIsStalledFiltered() {
        if (filteredCurrent() >= STALL_CURRENT) {
            stallLoops++;
        } else {
            stallLoops = 0;
        }
        return stallLoops >= REQUIRED_STALL_LOOPS;
    }

    public void checkStall() {
        double now = Timer.getFPGATimestamp();
        if (now - lastTime >= interval) {
            if (armIsStalledFiltered() == true) {
                armStalled = true;
            } else {
                armStalled = false;
            }
        }
    }

    public void timedArmUp() {
        // setIntake(INTAKE_SPEED);
        if (!armStalled) {
            UnpaidIntern.setPercentWithDeadband(motorB, ARM_UP_MOVE, DEADBAND);
            System.out.println("Not Stalled; Running motorB: "+ARM_UP_MOVE);
        } else {
            UnpaidIntern.setPercentWithDeadband(motorB, ARM_HOLD_STRONG, DEADBAND);
            System.out.println("Stalled; Testing motor movement for motorB: "+ARM_HOLD_STRONG); 
        }
        checkStall();
    }


    public void timedArmDown() {
        // setIntake(INTAKE_SPEED);
        RobotContainer.speedChange = 0.5;
        if (!armStalled) {
            UnpaidIntern.setPercentWithDeadband(motorB, -ARM_DOWN_MOVE, DEADBAND);
            System.out.println("Not Stalled; Running motorB: "+ARM_DOWN_MOVE);
        } else {
            UnpaidIntern.setPercentWithDeadband(motorB, -ARM_HOLD_STRONG, DEADBAND);  
            System.out.println("Stalled; Testing motor movement for motorB: "+ARM_HOLD_STRONG); 
        }
        checkStall();
    }

    public void stop() {
        setIntake(0);
        UnpaidIntern.stop(motorA);
        UnpaidIntern.stop(motorB);
        armStalled = false;
        stallLoops = 0;
        RobotContainer.speedChange = 1;
    }

    public void kill() {
        UnpaidIntern.killMotor(motorA);
        UnpaidIntern.killMotor(motorB);
        armStalled = false;
        stallLoops = 0;
    }

    public TalonFX getMotorA() { return motorA; }
    public TalonFX getMotorB() { return motorB; }
}