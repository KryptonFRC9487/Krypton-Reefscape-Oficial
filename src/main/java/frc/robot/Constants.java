package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;
import swervelib.parser.PIDFConfig;

/**
 * Classe Constants para armazenar constantes usadas no código do robô.
 * Organizada em classes internas para melhor separação de responsabilidades.
 */
public final class Constants {

  /**
   * Configurações relacionadas às dimensões e características físicas do robô.
   */
  public static final class Dimensoes {
    public static final double LOOP_TIME = 0.13; // Tempo de loop em segundos (sparkMax + normal)
    public static final double ROBOT_MASS = 38; // Massa do robô em quilogramas

    // Posições do centro de massa
    private static final double X_MASS = 0.0;
    private static final double Y_MASS = 0.0;
    private static final double Z_MASS = 0.08;

    // Centro de massa do chassi
    public static final Matter CHASSIS = new Matter(
        new Translation3d(X_MASS, Y_MASS, Z_MASS), ROBOT_MASS);

    // Máxima aceleração e velocidade (m/s² e m/s)
    public static final double MAX_ACCE_AUTO = 4.0;
    public static final double MAX_VEL_AUTO = 4.0;

    // Dimensões das rodas e reduções
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double DRIVE_GEAR_RATIO = 6.18;
    public static final double ANGLE_GEAR_RATIO = 21.42;

    // PPR do encoder interno NEO
    public static final double PULSE_PER_ROTATION = 1;

    // Fatores de conversão
    public static final double DRIVE_CONVERSION = SwerveMath.calculateMetersPerRotation(
        WHEEL_DIAMETER_METERS, DRIVE_GEAR_RATIO, PULSE_PER_ROTATION);
    public static final double ANGLE_CONVERSION = SwerveMath.calculateDegreesPerSteeringRotation(
        ANGLE_GEAR_RATIO, PULSE_PER_ROTATION);
  }

  /**
   * Configurações de PID para controle autônomo.
   */
  public static final class PID {
    public static final PIDFConfig X_AUTO_PID = new PIDFConfig(0.65, 0, 0.05);
    public static final PIDFConfig Y_AUTO_PID = new PIDFConfig(0.6, 0, 0);
    public static final PIDFConfig ANGLE_AUTO_PID = new PIDFConfig(0.1, 0, 0.01);
  }

  /**
   * Configurações relacionadas ao controle de tração.
   */
  public static final class SwerveMotionConfig {
    public static final boolean FIELD_RELATIVE = true; // Orientação ao campo
    public static final boolean IS_OPEN_LOOP = false; // Malha fechada
    public static final boolean ACCEL_CORRECTION = false; // Correção de aceleração

    // Multiplicadores para suavizar entradas do joystick
    public static final double MULT_ROTACIONAL = 0.8;
    public static final double MULT_TRANSLACIONAL_Y = 0.7;
    public static final double MULT_TRANSLACIONAL_X = 0.7;

    public static final double TURN_CONSTANT = 0.75; // Constante de rotação
    public static final double MAX_SPEED = 4.8; // Velocidade máxima (m/s)
    public static final double DT = 0.02; // Intervalo de tempo (s)
    public static final double CONSTANT_ROTATION = 4.0; // Rotação constante
  }

  /**
   * Configurações das trajetórias autônomas.
   */
  public static final class Trajetoria {
    public static final String NOME_TRAJETORIA1 = "New Auto";
    public static final String NOME_TRAJETORIA2 = "New Path2";
  }

  /**
   * Estados usados em strings.
   */
  public static final class StateStrings {
    public static final String OFF = "OFF";
    public static final String ON = "ON";
  }

  /**
   * Configurações do elevador.
   */
  public static final class ElevatorConstants {
    public static final int LEFT_MOTOR_ID = 16;
    public static final int RIGHT_MOTOR_ID = 15;
  }

  /**
   * Configurações do sistema de outtake.
   */
  public static final class OuttakeConstants {
    public static final int ANGLE_ID = 24;
    public static final int COLLECT_ID = 25;
  }

  /**
   * Configurações de visão.
   */
  public static final class ReefsVisionConstants {
    public static final int CAMERA_INDEX = 1;
    public static final String STREAM_NAME = "Red Detection";
    public static final int IMAGE_WIDTH = 640;
    public static final int IMAGE_HEIGHT = 480;
  }

  /**
   * Configurações do controle e botões.
   */
  public static final class GamepadConstants {
    public static final int P1_PORT = 0;
    public static final int P2_PORT = 1;
    public static final double DEADBAND = 0.2; // Deadband do controle P1
  }

  public static final class Buttons {
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int BUTTON_BACK = 7;
  }
}
