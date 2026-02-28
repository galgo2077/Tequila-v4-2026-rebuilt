package frc.robot.subsystems.Mates;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class angulo_shooter extends SubsystemBase {

    // --- PARÁMETROS DEL CAMPO (Basados en el manual) ---
    // La distancia al HUB es de 158.6 in desde la Alliance Wall
    private static final double DISTANCIA_OBJETIVO = 158.6; // se modificara con giro o limelight
    // La altura de la apertura es de 72 in
    private static final double ALTURA_OBJETIVO = 72.0; // se puede poner un poco mas alta para la proteccion de atras
    private static final double ALTURA_LANZADOR = 28.0; // se ajustara exactamente cuando el robot esté listo
    private static final double movimiento_minimo = 30;

    // iguales a la pelota
    private static final double GRAVEDAD = 386.1;
    private static final double PESO_PELOTA = 0.3;
    private static final double DIAMETRO_PELOTA = 9.5;
    private static final double MASA = PESO_PELOTA / GRAVEDAD;
    private static final double AREA = Math.PI * Math.pow(DIAMETRO_PELOTA / 2.0, 2);
    private static final double RHO_AIRE = 0.000044;
    private static final double CD = 0.47;

    public angulo_shooter() {
        // Inicializar valores en SmartDashboard si no existen
        SmartDashboard.putNumber("Shooters Velocity", 400.0);
    }

    @Override
    public void periodic() {
        // Obtenemos la velocidad actual configurada
        double velocidadActual = SmartDashboard.getNumber("Shooters Velocity", 0.0);

        // Calculamos el ángulo ideal basado en esa velocidad
        double anguloCalculado = buscarAngulo(velocidadActual);

        // Publicamos el resultado para que el driver lo vea o el motor lo use
        SmartDashboard.putNumber("Calculated Shooting Angle", anguloCalculado);
    }

    /**
     * Este método puede ser llamado por un comando para mover el motor
     */
    public double getTargetAngle() {
        double v = SmartDashboard.getNumber("Shooters Velocity", 0.0);
        return buscarAngulo(v);
    }

    private double simular(double anguloDeg, double v0) {
        double dt = 0.005;
        double x = 0;
        double y = ALTURA_LANZADOR;
        double vx = v0 * Math.cos(Math.toRadians(anguloDeg));
        double vy = v0 * Math.sin(Math.toRadians(anguloDeg));

        while (x < DISTANCIA_OBJETIVO) {
            double v = Math.sqrt(vx * vx + vy * vy);
            if (v == 0)
                break;

            double fd = 0.5 * RHO_AIRE * (v * v) * CD * AREA;
            double ax = -(fd * (vx / v)) / MASA;
            double ay = -GRAVEDAD - (fd * (vy / v)) / MASA;

            vx += ax * dt;
            vy += ay * dt;
            x += vx * dt;
            y += vy * dt;

            if (y < 0 && x < DISTANCIA_OBJETIVO)
                return -100;
        }
        return y;
    }

    private double buscarAngulo(double v0) {
        if (v0 <= 0)
            return 0;
        double mejorAngulo = 0;
        double menorError = Double.MAX_VALUE;

        for (double a = 20.0; a <= 80.0; a += 0.5) { // Paso de 0.5 para no saturar la CPU
            double alturaFinal = simular(a, v0);
            double error = Math.abs(alturaFinal - ALTURA_OBJETIVO);

            if (error < menorError) {
                menorError = error;
                mejorAngulo = a;
            }
        }
        return mejorAngulo;
    }

    // aca convierto mi angulo a algo que el shooter puede interpretar
    public double getrealangle(double mejorAngulo) {

        double angulo = mejorAngulo - movimiento_minimo;
        return angulo;
    }

}