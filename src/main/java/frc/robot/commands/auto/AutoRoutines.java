package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.mechanisms.SuperstructureCommand;

public class AutoRoutines {

    public static class BomberCenterAuto extends Command {
        private final SuperstructureCommand m_superstructure;
        public BomberCenterAuto(SuperstructureCommand superstructure) {
            this.m_superstructure = superstructure;
        }
        @Override public void initialize() {}
        @Override public void execute() {}
        @Override public boolean isFinished() { return true; }
        @Override public void end(boolean interrupted) {}
    }

    public static class BomberSprintAuto extends Command {
        private final SuperstructureCommand m_superstructure;
        public BomberSprintAuto(SuperstructureCommand superstructure) {
            this.m_superstructure = superstructure;
        }
        @Override public void initialize() {}
        @Override public void execute() {}
        @Override public boolean isFinished() { return true; }
        @Override public void end(boolean interrupted) {}
    }

    public static class StrikerFeedAuto extends Command {
        private final SuperstructureCommand m_superstructure;
        public StrikerFeedAuto(SuperstructureCommand superstructure) {
            this.m_superstructure = superstructure;
        }
        @Override public void initialize() {}
        @Override public void execute() {}
        @Override public boolean isFinished() { return true; }
        @Override public void end(boolean interrupted) {}
    }

    public static class InterceptorBlitzAuto extends Command {
        private final SuperstructureCommand m_superstructure;
        public InterceptorBlitzAuto(SuperstructureCommand superstructure) {
            this.m_superstructure = superstructure;
        }
        @Override public void initialize() {}
        @Override public void execute() {}
        @Override public boolean isFinished() { return true; }
        @Override public void end(boolean interrupted) {}
    }

    public static class TurretOnlyAuto extends Command {
        private final SuperstructureCommand m_superstructure;
        public TurretOnlyAuto(SuperstructureCommand superstructure) {
            this.m_superstructure = superstructure;
        }
        @Override public void initialize() {}
        @Override public void execute() {}
        @Override public boolean isFinished() { return true; }
        @Override public void end(boolean interrupted) {}
    }

    public static class JustLeaveAuto extends Command {
        private final SuperstructureCommand m_superstructure;
        public JustLeaveAuto(SuperstructureCommand superstructure) {
            this.m_superstructure = superstructure;
        }
        @Override public void initialize() {}
        @Override public void execute() {}
        @Override public boolean isFinished() { return true; }
        @Override public void end(boolean interrupted) {}
    }
}