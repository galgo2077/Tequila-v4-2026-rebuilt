package frc.robot.subsystems.Mates;

public class tiempo_angle {

    // El Kraken X44 a 12V
    private final double RPM_MAX = 7200.0;
    // Tu factor de conversión: 20 grados / 54 revs = 0.37 (aprox)
    private final double REVS_POR_GRADO = 0.369;
    private final double REVOLUCIONES_TOTALES = 54.0;

    /**
     * MÉTODO 1: Para el inicio (Reset o Calibración)
     * Calcula el tiempo para recorrer los 20 grados completos (54 revoluciones).
     */
    public double tiempo_inicio() {
        // (54 / 7200) * 60 = 0.45 segundos
        double revoluciones_necesarias = REVOLUCIONES_TOTALES / REVS_POR_GRADO;
        double tiempo = (revoluciones_necesarias / RPM_MAX) * 60.0;
        return tiempo;
    }

    /**
     * MÉTODO 2: Para posicionar durante el juego
     * 
     * @param grados_a_mover Cuántos grados faltan según tu sensor/cálculo
     */
    public double tiempo_angulo_posicionar(double grados_a_mover) {
        // Convertimos los grados que pides a revoluciones de motor
        double revoluciones_necesarias = grados_a_mover / REVS_POR_GRADO;

        // Calculamos cuánto tiempo debe estar encendido a 7200 RPM
        double tiempo = (revoluciones_necesarias / RPM_MAX) * 60.0;

        return tiempo;
    }
}