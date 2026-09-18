"""
Barrido automatico de API Class para el frame de "Set Reference".

Ya establecimos que:
- Clase 0 (duty_cycle) SI mueve el motor (confirmado desde antes).
- Clase 4 (hipotesis "max_motion_position") NO produce ninguna salida,
  ni en +12 ni en -12 rotaciones -- applied_output se queda en 0.000
  siempre. No es un problema mecanico ni de limit switch: el firmware
  simplemente no esta interpretando esos frames como un comando de
  movimiento.

Como REV no publica los valores de API Class por modo de control, este
script prueba varias clases candidatas UNA POR UNA y observa si
'applied_output' se mueve durante una ventana corta. Asi encontramos
empiricamente cual(es) clase(s) responden, en vez de seguir adivinando
a mano.

SEGURIDAD:
- Usa un setpoint pequeno por defecto (0.05) para minimizar el riesgo
  si alguna clase resulta interpretarse como duty cycle a full-scale
  por error de nuestra parte.
- Cada clase se prueba solo un par de segundos.
- Ten el motor libre de mecanismo cargado / listo para cortar
  alimentacion si algo se mueve de forma inesperada.
- NO se prueba la clase 0 (duty_cycle) por defecto ya que confirmaste
  que esa si mueve el motor -- no hace falta arriesgar con ella aqui.

Uso:
    python3 barrido_clases.py
    python3 barrido_clases.py --clases 1 2 3 4 5 6 7 --setpoint 0.05 --duracion 2.0
"""

import argparse
import time

from sparkmax import SparkMax


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can-id", type=int, default=1)
    parser.add_argument("--channel", type=str, default="can0")
    parser.add_argument("--clases", type=int, nargs="+",
                         default=[1, 2, 3, 4, 5, 6, 7],
                         help="Valores de API Class a probar, en orden")
    parser.add_argument("--setpoint", type=float, default=10,
                         help="Setpoint pequeno a usar en cada prueba")
    parser.add_argument("--duracion", type=float, default=10,    
                         help="Segundos a observar por cada clase")
    args = parser.parse_args()

    print("ADVERTENCIA: se van a probar varias API Class desconocidas.")
    print("Asegurate de que el motor pueda girar libremente sin riesgo")
    print("y de tener forma de cortar alimentacion si algo se mueve mal.")
    input("Presiona Enter para comenzar el barrido...")

    resultados = []

    with SparkMax(can_id=args.can_id, channel=args.channel) as motor:
        time.sleep(0.5)  # esperar primeras lecturas de telemetria

        for clase in args.clases:
            print(f"\n--- Probando API Class {clase} "
                  f"(setpoint={args.setpoint:+.3f}) ---")

            motor.set_raw_api_class(clase)
            motor.set_position(args.setpoint)

            max_applied = 0.0
            max_current = 0.0
            pos_antes = motor.get_position()

            t0 = time.monotonic()
            while time.monotonic() - t0 < args.duracion:
                ap = motor.get_applied_output()
                cur = motor.get_motor_current()
                max_applied = max(max_applied, abs(ap))
                max_current = max(max_current, abs(cur))
                time.sleep(0.05)

            pos_despues = motor.get_position()
            delta_pos = pos_despues - pos_antes

            # Volver a 0 / apagar salida antes de pasar a la siguiente clase
            motor.set_position(0.0)
            time.sleep(0.3)

            hubo_actividad = max_applied > 0.005 or max_current > 0.02 or abs(delta_pos) > 0.001
            resultados.append((clase, max_applied, max_current, delta_pos, hubo_actividad))

            marca = "  <-- POSIBLE MATCH" if hubo_actividad else ""
            print(f"    applied_max={max_applied:.3f}  I_max={max_current:.3f} A  "
                  f"delta_pos={delta_pos:+.4f} rot{marca}")

        motor.set_raw_api_class(None)
        motor.stop()

    print("\n" + "=" * 60)
    print("RESUMEN")
    print("=" * 60)
    for clase, ap, cur, dpos, actividad in resultados:
        estado = "ACTIVIDAD DETECTADA" if actividad else "sin respuesta"
        print(f"Clase {clase}: applied_max={ap:.3f}  I_max={cur:.3f} A  "
              f"delta_pos={dpos:+.4f} rot  -> {estado}")

    activas = [c for c, *_ , a in resultados if a]
    if activas:
        print(f"\nClase(s) candidata(s) a investigar mas: {activas}")
    else:
        print("\nNinguna clase probada produjo actividad medible.")
        print("Revisar: modo de control activo en el propio Spark Max, ")
        print("parametro de feedback sensor, o capturar trafico real de ")
        print("Hardware Client con candump (si Hardware Client usa el ")
        print("mismo bus CAN que estas escuchando, y no una conexion USB ")
        print("directa por separado al Spark Max).")


if __name__ == "__main__":
    main()