"""Ejemplo real de un eje; ejecutar sólo sobre el banco preparado para moverse."""
import argparse
import time
from teach_pendant_backend import TeachPendantBackend


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--id", type=int, default=1, dest="device_id")
    parser.add_argument("--target", "--setpoint", dest="target", type=float,
                        required=True, help="SP absoluto en rotaciones")
    parser.add_argument("--persist", action="store_true", help="Confirmar guardado en flash antes de mover")
    args = parser.parse_args()
    with TeachPendantBackend(channel=args.channel, device_id=args.device_id,
                             position_limits=(-2.0, 2.0)) as axis:
        # result() es conveniente en scripts. En una GUI: consultar done().
        axis.initialize().result(6)
        # P, I y perfil del ejemplo funcional adjunto. D y F se explicitan en cero.
        # Verificar/sintonizar para el mecanismo real antes de reutilizarlos.
        axis.set_pidf(p=1.0, i=0.0, d=0.0, f=0.0).result(3)
        axis.set_motion_profile(maxacceleration=500.0, cruisevelocity=900.0,
                                allowed_profile_error=0.1).result(3)
        if args.persist:
            axis.persist_parameters().result(4)

        deadline = time.monotonic() + 2.0
        while True:
            t = axis.telemetry()
            if t.fault:
                raise RuntimeError(t.fault)
            if t.position_fresh and t.current_fresh:
                break
            if time.monotonic() > deadline:
                raise TimeoutError("No hay telemetría fresca")
            time.sleep(0.02)

        axis.arm()  # SP inicial = PV; no ordena ir a cero.
        axis.send_setpoint(args.target)
        deadline = time.monotonic() + 15.0
        settled_since = None
        while time.monotonic() < deadline:
            t = axis.telemetry()
            if t.fault:
                raise RuntimeError(t.fault)
            print(f"SP={t.sp_rot:.4f} rot, PV={t.pv_rot:.4f} rot, "
                  f"v={t.velocity_rpm:.2f} RPM, I={t.current_a:.2f} A, "
                  f"error={t.error_rot if t.error_rot is not None else 'STALE'}")
            reached = (t.position_fresh and t.current_fresh and t.error_rot is not None
                       and abs(t.error_rot) < 0.01 and abs(t.velocity_rpm) < 1.0)
            settled_since = (settled_since or time.monotonic()) if reached else None
            if settled_since is not None and time.monotonic() - settled_since >= 0.25:
                print("Objetivo alcanzado según telemetría.")
                break
            time.sleep(0.1)  # Sólo el llamador espera; CAN continúa en su hilo.
        else:
            raise TimeoutError("No se alcanzó el objetivo en 15 s")
        axis.disarm()
        # Al salir del with también se deshabilita. No es retención mecánica.


if __name__ == "__main__":
    main()
