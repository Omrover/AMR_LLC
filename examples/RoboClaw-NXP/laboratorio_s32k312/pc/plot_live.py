"""Gráficas de la práctica; podemos abrir un ensayo sin conectar la placa."""
from cli import Parser
from collections import deque
from pathlib import Path
import math
from telemetry import read_csv


def build_figure(control=False):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import TextBox, Button
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
    fig.subplots_adjust(bottom=0.20 if control else 0.12, top=0.87, hspace=0.16)
    fig.suptitle("S32K312 + RoboClaw | Velocidad y posición", fontsize=16)
    specs = [
        ("speed_cps", 0, "Velocidad", "#0077b6", "-"),
        ("reference_cps", 0, "Referencia rampada", "#e76f51", "-"),
        ("target_cps", 0, "Consigna", "#777777", "--"),
        ("duty_pct", 1, "Duty solicitado NXP", "#2a9d8f", "-"),
        ("position_counts", 2, "Posición / encoder", "#6a4c93", "-"),
        ("position_target_counts", 2, "Destino de posición", "#e76f51", "--"),
    ]
    lines = {}
    for field, axis, label, color, style in specs:
        lines[field], = axes[axis].plot([], [], style, label=label, color=color, linewidth=1.6)
    for axis, unit in zip(axes, ["Conteos/s", "Duty (%)", "Conteos"]):
        axis.set_ylabel(unit)
        axis.grid(alpha=0.25)
        axis.legend(loc="upper left", fontsize=9)
    axes[-1].set_xlabel("Tiempo de recepcion PC (s)")
    info = fig.text(0.10, 0.905, "Esperando telemetria...", fontsize=10)
    message = fig.text(0.10, 0.03, "", fontsize=9)
    controls = None
    if control:
        box = TextBox(fig.add_axes([0.15, 0.10, 0.55, 0.05]), "Comando ")
        stop = Button(fig.add_axes([0.75, 0.10, 0.15, 0.05]), "STOP", color="#ffc9c9")
        controls = (box, stop)
    else:
        message.set_text("Solo lectura. Duty es una orden NXP, no PWM medido; NA se deja sin graficar.")
    return fig, axes, lines, info, message, controls


def refresh(history, axes, lines, info, window):
    if not history:
        return
    times = [item[0] for item in history]
    for field, line in lines.items():
        values = [getattr(sample, field) for _, sample in history]
        line.set_data(times, [float("nan") if value is None else value for value in values])
    for axis in axes:
        axis.relim()
        axis.autoscale_view()
    right = max(times[-1], 1.0)
    axes[-1].set_xlim(max(0.0, right - window), right)
    sample = history[-1][1]
    info.set_text(
        f"muestra={sample.n} | velocidad={sample.speed_cps} conteos/s | "
        f"Vbat={sample.battery_v} V | dt={sample.dt_us} us | "
        f"estado={sample.position_status},{sample.speed_status},{sample.voltage_status}"
    )


def save_figure(fig, path):
    if path:
        with open(path, "xb") as stream:
            fig.savefig(stream, format="png", dpi=140)


def main():
    parser = Parser(description=__doc__)
    parser.add_argument("port", nargs="?", help="COM OpenSDA NXP")
    parser.add_argument("--control", action="store_true", help="Activar comandos y STOP al cerrar")
    parser.add_argument("--log", help="CSV nuevo durante lectura serial")
    parser.add_argument("--window", type=float, default=30, help="Ventana visible en segundos")
    parser.add_argument("--replay", help="CSV grabado; no abre serial")
    parser.add_argument("--save", help="PNG nuevo, no sobrescribir")
    parser.add_argument("--no-show", action="store_true", help="Solo exportar replay a PNG")
    args = parser.parse_args()
    if not math.isfinite(args.window) or not 0 < args.window <= 3600:
        parser.error("--window debe ser >0 y <=3600.")
    if args.save and Path(args.save).exists():
        parser.error("El PNG ya existe; elegir otro nombre.")
    if args.replay and (args.port or args.control or args.log):
        parser.error("--replay no se combina con puerto, --control o --log.")
    if args.no_show and (not args.replay or not args.save):
        parser.error("--no-show requiere --replay y --save.")
    if not args.replay and not args.port:
        parser.error("Indicar puerto o --replay.")
    if not args.replay:
        from panel import run_panel
        run_panel(args.port, control=args.control, log=args.log,
                  window=args.window, save=args.save)
        return
    if args.no_show:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes, lines, info, message, controls = build_figure(args.control)
    # Limitamos la pantalla; el CSV conserva cada muestra válida.
    history = deque(maxlen=20000)
    if args.replay:
        history.extend(read_csv(args.replay))
        if not history:
            raise ValueError("CSV sin telemetria.")
        refresh(history, axes, lines, info, args.window)
        message.set_text(f"Ensayo: {Path(args.replay).name} | tiempo de recepcion PC, no reloj NXP.")
        save_figure(fig, args.save)
        if not args.no_show:
            plt.show()
        plt.close(fig)
        return

if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ImportError, ValueError) as exc:
        raise SystemExit(str(exc))
