"""Terminal interactiva para TeachPendantBackend; la E/S CAN usa su propio hilo."""
from __future__ import annotations

import argparse
import cmd
from dataclasses import asdict
import json
import shlex
import threading

from teach_pendant_backend import DEFAULT_SPEC, TeachPendantBackend


class PendantTerminal(cmd.Cmd):
    prompt = "pendant> "
    intro = (
        "MAXMotion Position / un eje. Escribe help para ver comandos.\n"
        "Configura pidf y profile, revisa jobs y status, y después usa arm y move."
    )

    def __init__(self, backend):
        super().__init__()
        self.axis = backend
        self.jobs = {}
        self.counter = 0
        self._watch_stop = threading.Event()
        self._watch_thread = None

    def onecmd(self, line):
        try:
            return super().onecmd(line)
        except (ValueError, RuntimeError, TypeError) as exc:
            print(f"Error: {exc}")
            return False

    def emptyline(self):
        pass  # Enter nunca repite un movimiento.

    @staticmethod
    def numbers(line, minimum, maximum=None):
        tokens = shlex.split(line)
        maximum = minimum if maximum is None else maximum
        if not minimum <= len(tokens) <= maximum:
            raise ValueError(f"Se requieren {minimum}..{maximum} argumentos numéricos")
        return [float(x) for x in tokens]

    def track(self, label, ticket):
        self.counter += 1
        self.jobs[self.counter] = (label, ticket)
        print(f"#{self.counter}: {label} encolado. Consulta jobs para ver confirmación.")

    def do_pidf(self, line):
        "pidf P I D F: actualizar las cuatro ganancias, con eje desarmado."
        p, i, d, f = self.numbers(line, 4)
        self.track("PIDF", self.axis.set_pidf(p=p, i=i, d=d, f=f))

    def do_profile(self, line):
        "profile ACEL_RPM_S CRUCERO_RPM [ERROR_ROT]: configurar MAXMotion."
        values = self.numbers(line, 2, 3)
        self.track("Perfil", self.axis.set_motion_profile(
            maxacceleration=values[0], cruisevelocity=values[1],
            allowed_profile_error=values[2] if len(values) == 3 else None))

    def do_accel(self, line):
        "accel RPM_S: actualizar aceleración máxima, con eje desarmado."
        self.track("Aceleración", self.axis.set_maxacceleration(self.numbers(line, 1)[0]))

    def do_cruise(self, line):
        "cruise RPM: actualizar velocidad de crucero, con eje desarmado."
        self.track("Crucero", self.axis.set_cruisevelocity(self.numbers(line, 1)[0]))

    def do_save(self, _line):
        "save: persistir parámetros en flash y verificar RESULT_CODE=0, desarmado."
        self.track("Persistencia", self.axis.persist_parameters())

    def do_jobs(self, _line):
        "jobs: consultar respuestas ACK y errores sin esperar al bus CAN."
        completed = []
        for number, (label, ticket) in self.jobs.items():
            if not ticket.done():
                print(f"#{number} {label}: pendiente")
                continue
            try:
                result = ticket.result()
                print(f"#{number} {label}: OK, {len(result)} respuestas verificadas")
            except Exception as exc:
                print(f"#{number} {label}: ERROR: {exc}")
            completed.append(number)
        for number in completed:
            del self.jobs[number]

    def do_arm(self, _line):
        "arm: habilitar el eje manteniendo la PV actual."
        print(f"Eje habilitado; SP inicial = {self.axis.arm():.6f} rot")

    def do_move(self, line):
        "move ROTACIONES: enviar una posición ABSOLUTA mediante MAXMotion."
        sp = self.axis.send_setpoint(self.numbers(line, 1)[0])
        print(f"SP publicado: {sp:.6f} rot")

    def do_hold(self, _line):
        "hold: cambiar SP a la PV actual usando el mismo perfil MAXMotion."
        print(f"Objetivo de retención: {self.axis.hold():.6f} rot")

    def do_disarm(self, _line):
        "disarm: detener heartbeat/SP; el watchdog del SPARK deshabilita el eje."
        self.axis.disarm()
        print("Paro de heartbeat solicitado; la deshabilitación depende del watchdog del SPARK")

    def do_status(self, _line):
        "status: SP, PV, RPM, amperes, error, frescura y estado del backend."
        print(json.dumps(asdict(self.axis.telemetry()), indent=2, ensure_ascii=False))

    def _watch(self, hz):
        while not self._watch_stop.wait(1.0 / hz):
            t = self.axis.telemetry()
            def fmt(v):
                return "--" if v is None else f"{v:.4f}"
            print(f"\nSP={fmt(t.sp_rot)} rot | PV={fmt(t.pv_rot)} rot | "
                  f"v={fmt(t.velocity_rpm)} RPM | I={fmt(t.current_a)} A | "
                  f"e={fmt(t.error_rot)} rot | "
                  f"fresh={t.position_fresh and t.current_fresh} | "
                  f"armed={t.armed} | fault={t.fault}", flush=True)

    def stop_watch(self):
        self._watch_stop.set()
        if self._watch_thread is not None:
            self._watch_thread.join(timeout=1.0)
            self._watch_thread = None

    def do_watch(self, line):
        "watch on [HZ] / watch off: telemetría en otro hilo; aún puedes escribir comandos."
        parts = shlex.split(line)
        if parts == ["off"]:
            self.stop_watch()
            return
        if not parts or parts[0] != "on" or len(parts) > 2:
            raise ValueError("Uso: watch on [HZ] o watch off")
        hz = float(parts[1]) if len(parts) == 2 else 2.0
        if not 0.2 <= hz <= 20:
            raise ValueError("HZ debe estar entre 0.2 y 20")
        self.stop_watch()
        self._watch_stop.clear()
        self._watch_thread = threading.Thread(target=self._watch, args=(hz,), daemon=True)
        self._watch_thread.start()

    def do_quit(self, _line):
        "quit: detener heartbeat, cerrar CAN y salir. Ctrl+C también cierra."
        self.axis.disarm()
        return True

    do_exit = do_quit
    do_EOF = do_quit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--id", type=int, default=1, dest="device_id")
    parser.add_argument("--slot", type=int, choices=range(4), default=0)
    parser.add_argument("--spec", default=str(DEFAULT_SPEC))
    parser.add_argument("--min-rot", type=float)
    parser.add_argument("--max-rot", type=float)
    args = parser.parse_args()
    if (args.min_rot is None) != (args.max_rot is None):
        parser.error("Usa --min-rot y --max-rot juntos")
    limits = None if args.min_rot is None else (args.min_rot, args.max_rot)
    terminal = None
    try:
        with TeachPendantBackend(args.spec, device_id=args.device_id,
                                 channel=args.channel, interface=args.interface,
                                 slot=args.slot, position_limits=limits) as axis:
            print("Inicializando encoder, unidades y telemetría (eje deshabilitado)...")
            axis.initialize().result(timeout=6.0)
            terminal = PendantTerminal(axis)
            try:
                terminal.cmdloop()
            finally:
                terminal.stop_watch()
    except KeyboardInterrupt:
        print("\nCierre solicitado por teclado.")
    except Exception as exc:
        parser.exit(1, f"Error: {exc}\n")


if __name__ == "__main__":
    main()
