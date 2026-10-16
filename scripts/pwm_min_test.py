#!/usr/bin/env python3
"""
Lanza la prueba de PWM minimo en la STM (firmware stm_pwm_min_test), registra
todo en CSV y al final imprime un resumen con el PWM minimo de cada rueda.

Dos valores por rueda y sentido:
  - pwm_start: PWM minimo para ARRANCAR desde el reposo (friccion estatica).
  - pwm_stop : PWM minimo para SEGUIR girando una vez en movimiento
               (friccion dinamica). Es el que importa cuando el control de
               trayectoria baja la velocidad cerca del objetivo.

Ejemplos:
  # Cada rueda por separado, ambos sentidos (robot levantado o en el piso)
  python3 pwm_min_test.py --mode single

  # Solo FR y BL, sentido positivo, 5 repeticiones
  python3 pwm_min_test.py --mode single --wheels FR,BL --dir pos --trials 5

  # Robot en el piso, las 4 ruedas juntas avanzando/retrocediendo
  python3 pwm_min_test.py --mode group --pattern forward

  # Robot en el piso, desplazamiento lateral (rodillos = mas friccion)
  python3 pwm_min_test.py --mode group --pattern lateral

Requiere el micro-ROS Agent corriendo, p. ej.:
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
"""

import argparse
import csv
import os
import statistics
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray

WHEELS = ["FR", "FL", "BR", "BL"]
PATTERNS = {"forward": 0, "lateral": 1, "rotation": 2}
PATTERN_NAMES = {v: k for k, v in PATTERNS.items()}
DIRS = {"both": 0, "pos": 1, "neg": -1}

STATE_NAMES = {
    0: "IDLE", 1: "REST", 2: "RAMP_UP", 3: "BOOST",
    4: "RAMP_DOWN", 5: "DONE", 6: "ABORTED", 7: "REJECTED",
}
ST_IDLE, ST_DONE, ST_ABORTED, ST_REJECTED = 0, 5, 6, 7

FLAG_START_OK, FLAG_STOP_OK, FLAG_WRONG_DIR = 0x01, 0x02, 0x04

# Mismos parametros que el firmware (Core/Src/pwm_test.c / control_PID.c).
PWM_MAX = 999.0
V_SUPPLY = 12.0


def pwm_to_duty(pwm):
    return 100.0 * pwm / PWM_MAX


def pwm_to_volts(pwm):
    return V_SUPPLY * pwm / PWM_MAX


class PwmMinTester(Node):
    def __init__(self, out_dir):
        super().__init__("pwm_min_tester")
        self.cmd_pub = self.create_publisher(Int32MultiArray, "/pwm_test/cmd", 10)
        self.create_subscription(Float64MultiArray, "/pwm_test/status", self.on_status, 50)
        self.create_subscription(Float64MultiArray, "/pwm_test/result", self.on_result, 50)

        self.last_status = None
        self.last_status_time = None
        self.results = []

        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.steps_path = os.path.join(out_dir, "pasos_%s.csv" % stamp)
        self.results_path = os.path.join(out_dir, "resultados_%s.csv" % stamp)

        self.steps_file = open(self.steps_path, "w", newline="")
        self.steps_csv = csv.writer(self.steps_file)
        self.steps_csv.writerow(["t", "state", "mode", "target", "dir", "trial", "pwm",
                                 "vel_fr", "vel_fl", "vel_br", "vel_bl"])

        self.results_file = open(self.results_path, "w", newline="")
        self.results_csv = csv.writer(self.results_file)
        self.results_csv.writerow(["mode", "pattern", "wheel", "dir", "trial", "pwm_start",
                                   "pwm_stop", "vel_start", "vel_stop", "flags"])
        self.t0 = time.time()

    def send(self, data):
        msg = Int32MultiArray()
        msg.data = [int(x) for x in data]
        self.cmd_pub.publish(msg)

    def on_status(self, msg):
        d = list(msg.data)
        if len(d) < 10:
            return
        self.last_status = d
        self.last_status_time = time.time()
        state = int(d[0])
        self.steps_csv.writerow(["%.3f" % (time.time() - self.t0)] + d)

        if state in (2, 4):
            vels = " ".join("%s=%+6.2f" % (WHEELS[i], d[6 + i]) for i in range(4))
            print("  %-9s trial %d  dir %+d  pwm %4d  | %s" % (
                STATE_NAMES[state], int(d[4]), int(d[3]), int(d[5]), vels))

    def on_result(self, msg):
        d = list(msg.data)
        if len(d) < 10:
            return
        self.results.append(d)
        self.results_csv.writerow(d)
        self.results_file.flush()

        wheel, direction, trial, flags = int(d[2]), int(d[3]), int(d[4]), int(d[9])
        kind = "grueso" if trial == 0 else "fino %d" % trial
        start = ("%d" % d[5]) if flags & FLAG_START_OK else "no arranco"
        stop = ("%d" % d[6]) if flags & FLAG_STOP_OK else "invalido"
        extra = "  (!) giro en sentido contrario" if flags & FLAG_WRONG_DIR else ""
        print(">> RESULTADO %s dir %+d (%s): arranque=%s  sostiene=%s%s" % (
            WHEELS[wheel], direction, kind, start, stop, extra))

    def close(self):
        self.steps_file.close()
        self.results_file.close()


def spin_for(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def summarize(results):
    """Agrupa por (rueda, sentido) usando los intentos finos (o el grueso si no hay)."""
    groups = {}
    for r in results:
        key = (int(r[2]), int(r[3]))
        groups.setdefault(key, []).append(r)

    print("\n" + "=" * 78)
    print("RESUMEN (PWM en cuentas 0..999; V = equivalente sobre %.0f V)" % V_SUPPLY)
    print("=" * 78)
    print("%-4s %-4s | %-22s | %-22s | %s" % ("Rued", "Dir", "ARRANQUE (max / media)",
                                              "SOSTIENE (max / media)", "vel min"))
    print("-" * 78)

    per_wheel_stop = {}
    per_wheel_start = {}
    per_wheel_vel = {}
    for (wheel, direction) in sorted(groups):
        rows = groups[(wheel, direction)]
        fine = [r for r in rows if int(r[4]) > 0]
        use = fine if fine else rows
        starts = [r[5] for r in use if int(r[9]) & FLAG_START_OK]
        stops = [r[6] for r in use if int(r[9]) & FLAG_STOP_OK]

        def fmt(vals):
            if not vals:
                return "sin datos"
            return "%3d / %5.1f (%.2f V)" % (max(vals), statistics.mean(vals), pwm_to_volts(max(vals)))

        stop_vels = [abs(r[8]) for r in use if int(r[9]) & FLAG_STOP_OK]
        vel_txt = ("%.2f rad/s" % statistics.mean(stop_vels)) if stop_vels else "-"
        print("%-4s %+-4d | %-22s | %-22s | %s" % (
            WHEELS[wheel], direction, fmt(starts), fmt(stops), vel_txt))

        if stop_vels:
            per_wheel_vel[wheel] = max(per_wheel_vel.get(wheel, 0.0), statistics.mean(stop_vels))

        if stops:
            per_wheel_stop[wheel] = max(per_wheel_stop.get(wheel, 0), max(stops))
        if starts:
            per_wheel_start[wheel] = max(per_wheel_start.get(wheel, 0), max(starts))

    if not per_wheel_stop:
        print("\nNo hay mediciones validas de 'sostiene'.")
        return

    print("-" * 78)
    print("\nPWM minimo recomendado por rueda (peor caso entre sentidos e intentos):")
    for wheel in sorted(per_wheel_stop):
        s = per_wheel_stop[wheel]
        a = per_wheel_start.get(wheel, float("nan"))
        print("  %s: sostiene %3d (%.1f %%, %.2f V)   arranca %3.0f (%.1f %%, %.2f V)" % (
            WHEELS[wheel], s, pwm_to_duty(s), pwm_to_volts(s), a, pwm_to_duty(a), pwm_to_volts(a)))

    robot_stop = max(per_wheel_stop.values())
    print("\nRobot completo (rueda mas exigente): sostiene %d cuentas = %.2f V" % (
        robot_stop, pwm_to_volts(robot_stop)))

    if per_wheel_vel:
        print("Velocidad minima sostenible de rueda: %.2f rad/s (por debajo de esto la"
              " rueda se frena aunque se le mande PWM)" % max(per_wheel_vel.values()))

    print("\nValores para control_PID.c (voltaje minimo por rueda, orden FR, FL, BR, BL):")
    print("  static const float U_MIN_WHEEL[4] = { "
          + ", ".join("%.2ff" % pwm_to_volts(per_wheel_stop.get(i, robot_stop)) for i in range(4))
          + " };")
    print("  Ver README.md del proyecto para como usarlos.")


def main():
    parser = argparse.ArgumentParser(description="Prueba de PWM minimo de los motores")
    parser.add_argument("--mode", choices=["single", "group"], default="single")
    parser.add_argument("--wheels", default="FR,FL,BR,BL",
                        help="Modo single: ruedas a probar, separadas por coma")
    parser.add_argument("--pattern", choices=list(PATTERNS), default="forward",
                        help="Modo group: patron de movimiento")
    parser.add_argument("--dir", choices=list(DIRS), default="both")
    parser.add_argument("--trials", type=int, default=3, help="Repeticiones finas (1..10)")
    parser.add_argument("--limit", type=int, default=600, help="PWM maximo permitido (cuentas)")
    parser.add_argument("--out", default="resultados_pwm", help="Carpeta para los CSV")
    args = parser.parse_args()

    if args.mode == "single":
        mask = 0
        for name in args.wheels.upper().split(","):
            name = name.strip()
            if name not in WHEELS:
                parser.error("rueda desconocida: %s" % name)
            mask |= 1 << WHEELS.index(name)
        cmd = [1, mask, DIRS[args.dir], args.trials, args.limit]
    else:
        cmd = [2, PATTERNS[args.pattern], DIRS[args.dir], args.trials, args.limit]

    rclpy.init()
    node = PwmMinTester(args.out)
    finished_state = None

    try:
        print("Esperando latido de la STM en /pwm_test/status ...")
        deadline = time.time() + 15.0
        while node.last_status is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.last_status is None:
            print("No llega /pwm_test/status. Revisa el micro-ROS Agent y la STM.")
            return

        # Se reenvia el comando hasta ver que la STM salio de IDLE.
        started = False
        for _ in range(5):
            node.send(cmd)
            t_sent = time.time()
            while time.time() - t_sent < 2.0:
                rclpy.spin_once(node, timeout_sec=0.05)
                if node.last_status_time and node.last_status_time > t_sent:
                    state = int(node.last_status[0])
                    if state == ST_REJECTED:
                        print("La STM rechazo el comando %s" % cmd)
                        return
                    if state not in (ST_IDLE, ST_DONE, ST_ABORTED):
                        started = True
                        break
            if started:
                break
        if not started:
            print("La STM no inicio la prueba.")
            return

        print("Prueba iniciada: %s  (Ctrl+C para abortar)" % cmd)
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            state = int(node.last_status[0])
            if state in (ST_DONE, ST_ABORTED):
                finished_state = state
                break
            if time.time() - node.last_status_time > 5.0:
                print("Se perdio la comunicacion con la STM.")
                break

        # Da tiempo a que lleguen los ultimos resultados.
        spin_for(node, 0.5)
        print("\nEstado final: %s" % STATE_NAMES.get(finished_state, "desconocido"))
        summarize(node.results)
    except KeyboardInterrupt:
        print("\nAbortando prueba...")
        for _ in range(5):
            node.send([0])
            spin_for(node, 0.1)
        summarize(node.results)
    finally:
        print("\nCSV de pasos:      %s" % node.steps_path)
        print("CSV de resultados: %s" % node.results_path)
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
