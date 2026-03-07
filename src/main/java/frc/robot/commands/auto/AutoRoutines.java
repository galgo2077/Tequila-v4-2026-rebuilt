package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.mechanisms.SuperstructureCommand;
import frc.robot.commands.mechanisms.SuperstructureCommand.DriveMode;
import frc.robot.commands.mechanisms.SuperstructureCommand.State;

// ═══════════════════════════════════════════════════════════════════════════
//  BomberCenterAuto — 4 bolas, ambos shooters, posición central
//
//  Path: BomberCenter.auto
//  Modo: BOMBER
//  Descripción:
//    - Preloaded → score inmediato con ambos shooters
//    - Recoger 3 bolas del campo mientras el turret sigue apuntando
//    - Disparar en cada parada
//  Expected output: 4 bolas, máximo puntaje
//
//  Named Events en PathPlanner:
//    t=0.0s  → PrepDual        (ramp shooters con preloaded)
//    t=1.2s  → ShootDual       (disparar preloaded)
//    t=2.0s  → IntakeDown      (recoger ball 1)
//    t=4.5s  → IntakeUp + PrepDual
//    t=5.5s  → ShootDual       (ball 1)
//    t=6.5s  → IntakeDown      (recoger balls 2+3)
//    t=9.0s  → IntakeUp + PrepDual
//    t=10.0s → ShootDual       (balls 2+3)
//    t=11.0s → StopShoot
// ═══════════════════════════════════════════════════════════════════════════
class BomberCenterAuto implements Command {

    private final Command m_command;

    BomberCenterAuto(SuperstructureCommand superstructure) {
        m_command = Commands.sequence(
            // Establecer modo BOMBER antes de que arranque el path
            Commands.runOnce(() -> superstructure.setDriveMode(DriveMode.BOMBER)),
            Commands.runOnce(() -> superstructure.requestState(State.IDLE)),
            // PathPlanner ejecuta el path + named events automáticamente
            AutoBuilder.buildAuto("BomberCenter")
        );
    }

    @Override public void initialize()            { m_command.initialize(); }
    @Override public void execute()               { m_command.execute(); }
    @Override public boolean isFinished()         { return m_command.isFinished(); }
    @Override public void end(boolean interrupted){ m_command.end(interrupted); }
}

// ═══════════════════════════════════════════════════════════════════════════
//  BomberSprintAuto — 2 bolas, velocidad máxima
//
//  Path: BomberSprint.auto
//  Modo: BOMBER
//  Descripción:
//    - Preloaded + 1 ball, ambos shooters, path corto y agresivo
//    - Prioriza consistencia sobre cantidad
//  Expected output: 2 bolas, tiempo ciclo <8s
//
//  Named Events:
//    t=0.0s  → PrepDual
//    t=1.0s  → ShootDual
//    t=2.0s  → IntakeDown
//    t=5.0s  → IntakeUp + PrepDual
//    t=6.0s  → ShootDual
//    t=7.0s  → StopShoot
// ═══════════════════════════════════════════════════════════════════════════
class BomberSprintAuto implements Command {

    private final Command m_command;

    BomberSprintAuto(SuperstructureCommand superstructure) {
        m_command = Commands.sequence(
            Commands.runOnce(() -> superstructure.setDriveMode(DriveMode.BOMBER)),
            Commands.runOnce(() -> superstructure.requestState(State.IDLE)),
            AutoBuilder.buildAuto("BomberSprint")
        );
    }

    @Override public void initialize()            { m_command.initialize(); }
    @Override public void execute()               { m_command.execute(); }
    @Override public boolean isFinished()         { return m_command.isFinished(); }
    @Override public void end(boolean interrupted){ m_command.end(interrupted); }
}

// ═══════════════════════════════════════════════════════════════════════════
//  StrikerFeedAuto — travesía completa, feed a alianza
//
//  Path: StrikerFeed.auto
//  Modo: STRIKER
//  Descripción:
//    - Sin disparos propios — recoge piezas y las pasa a aliados
//    - Maximiza el output de aliados en Striker feeding strategy
//  Expected output: 0 puntos propios, máximo feed a alianza
//
//  Named Events:
//    t=0.0s  → IntakeDown
//    t=3.0s  → FeedOut         (empieza a pasar)
//    t=12.0s → IntakeDown      (segunda pasada)
//    t=14.0s → StopShoot
// ═══════════════════════════════════════════════════════════════════════════
class StrikerFeedAuto implements Command {

    private final Command m_command;

    StrikerFeedAuto(SuperstructureCommand superstructure) {
        m_command = Commands.sequence(
            Commands.runOnce(() -> superstructure.setDriveMode(DriveMode.STRIKER)),
            Commands.runOnce(() -> superstructure.requestState(State.IDLE)),
            AutoBuilder.buildAuto("StrikerFeed")
        );
    }

    @Override public void initialize()            { m_command.initialize(); }
    @Override public void execute()               { m_command.execute(); }
    @Override public boolean isFinished()         { return m_command.isFinished(); }
    @Override public void end(boolean interrupted){ m_command.end(interrupted); }
}

// ═══════════════════════════════════════════════════════════════════════════
//  InterceptorBlitzAuto — cruzar campo, robar bolas rivales
//
//  Path: InterceptorBlitz.auto
//  Modo: INTERCEPTOR (80% velocidad)
//  Descripción:
//    - Cruzar al campo rival, posicionarse en zona de bolas
//    - Bloquear colección rival con posición y movimiento
//    - No dispara — objetivo defensivo
//  Expected output: disrupción defensiva
//
//  Named Events:
//    t=0.0s  → IntakeDown      (preparar para interceptar)
//    t=8.0s  → IntakeUp        (retracting para no dañar al cruzar)
//    t=10.0s → IntakeDown
//    t=14.0s → StopShoot
// ═══════════════════════════════════════════════════════════════════════════
class InterceptorBlitzAuto implements Command {

    private final Command m_command;

    InterceptorBlitzAuto(SuperstructureCommand superstructure) {
        m_command = Commands.sequence(
            Commands.runOnce(() -> superstructure.setDriveMode(DriveMode.INTERCEPTOR)),
            Commands.runOnce(() -> superstructure.requestState(State.IDLE)),
            AutoBuilder.buildAuto("InterceptorBlitz")
        );
    }

    @Override public void initialize()            { m_command.initialize(); }
    @Override public void execute()               { m_command.execute(); }
    @Override public boolean isFinished()         { return m_command.isFinished(); }
    @Override public void end(boolean interrupted){ m_command.end(interrupted); }
}

// ═══════════════════════════════════════════════════════════════════════════
//  TurretOnlyAuto — 3 bolas conservador, solo turret móvil
//
//  Path: TurretOnly.auto
//  Modo: STRIKER
//  Descripción:
//    - Fallback confiable si el fixed shooter tiene problemas
//    - 3 bolas con turret solo, paths más lentos y precisos
//  Expected output: 3 bolas, alta consistencia
//
//  Named Events:
//    t=0.0s  → PrepTurret
//    t=1.2s  → ShootTurret
//    t=2.0s  → IntakeDown
//    t=5.0s  → IntakeUp + PrepTurret
//    t=6.0s  → ShootTurret
//    t=7.5s  → IntakeDown
//    t=10.0s → IntakeUp + PrepTurret
//    t=11.0s → ShootTurret
//    t=12.0s → StopShoot
// ═══════════════════════════════════════════════════════════════════════════
class TurretOnlyAuto implements Command {

    private final Command m_command;

    TurretOnlyAuto(SuperstructureCommand superstructure) {
        m_command = Commands.sequence(
            Commands.runOnce(() -> superstructure.setDriveMode(DriveMode.STRIKER)),
            Commands.runOnce(() -> superstructure.requestState(State.IDLE)),
            AutoBuilder.buildAuto("TurretOnly")
        );
    }

    @Override public void initialize()            { m_command.initialize(); }
    @Override public void execute()               { m_command.execute(); }
    @Override public boolean isFinished()         { return m_command.isFinished(); }
    @Override public void end(boolean interrupted){ m_command.end(interrupted); }
}

// ═══════════════════════════════════════════════════════════════════════════
//  JustLeaveAuto — solo salir de la línea de inicio
//
//  Path: JustLeave.auto  (o inline — 1m hacia adelante)
//  Modo: BOMBER (default)
//  Descripción:
//    - Emergencia — garantiza puntos de movilidad sin importar nada
//    - Sin mecanismos, sin colección, sin disparo
//  Expected output: puntos de movilidad (salir de la línea)
// ═══════════════════════════════════════════════════════════════════════════
class JustLeaveAuto implements Command {

    private final Command m_command;

    JustLeaveAuto(SuperstructureCommand superstructure) {
        m_command = Commands.sequence(
            Commands.runOnce(() -> superstructure.setDriveMode(DriveMode.BOMBER)),
            Commands.runOnce(() -> superstructure.requestState(State.IDLE)),
            // Usa el .auto de PathPlanner si existe; si no, drive inline
            AutoBuilder.buildAuto("JustLeave")
        );
    }

    @Override public void initialize()            { m_command.initialize(); }
    @Override public void execute()               { m_command.execute(); }
    @Override public boolean isFinished()         { return m_command.isFinished(); }
    @Override public void end(boolean interrupted){ m_command.end(interrupted); }
}
