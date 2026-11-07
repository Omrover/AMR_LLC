"""Panel de velocidad y posición: lectura, límites y ajuste del control NXP."""
from cli import Parser
from collections import deque
from decimal import Decimal, InvalidOperation
import math
import queue
import threading
import time
from telemetry import (normalize_command, parse_line, counts_to_rpm,
                       DEFAULT_COUNTS_PER_REV, INT32_MIN, INT32_MAX)


def gain_command(values, command="PID", maximum=1):
    """En el panel escribimos 0.020; la placa recibe 20 milésimas."""
    converted = []
    for value in values:
        try:
            gain = Decimal(str(value))
            milli = gain * 1000
            if not gain.is_finite() or not 0 <= gain <= maximum or milli != milli.to_integral_value():
                raise ValueError
            converted.append(int(milli))
        except (InvalidOperation, ValueError, OverflowError) as exc:
            raise ValueError(f"Ganancias entre 0 y {maximum}, con hasta tres decimales.") from exc
    return normalize_command(command + " " + " ".join(map(str, converted)))


def parse_config(line):
    if not line.startswith("CONFIG "):
        return None
    try:
        values = {key: int(value) for key, value in
                  (item.split("=", 1) for item in line.split()[1:])}
        required = {"protocolo", "etapa", "minimo", "maximo", "kp_milesimas",
                    "ki_milesimas", "kd_milesimas", "armado"}
        if not required.issubset(values) or values["protocolo"] != 2:
            return None
        normalize_command(f"LIMITES {values['minimo']} {values['maximo']}")
        normalize_command("PID " + " ".join(str(values[key]) for key in
                          ("kp_milesimas", "ki_milesimas", "kd_milesimas")))
        if "duty_maximo" in values:
            normalize_command(f"DUTYMAX {values['duty_maximo']}")
        if values["etapa"] not in (1, 2, 3, 4, 5) or values["armado"] not in (0, 1):
            return None
        if values["etapa"] == 5:
            if not {"cero_definido", "origen", "ventana", "pos_min", "pos_max", "pos_kp_milesimas",
                    "pos_kd_milesimas", "tolerancia", "duty_maximo"}.issubset(values):
                return None
            if values["cero_definido"] not in (0, 1) or values["ventana"] not in (0, 1):
                return None
            normalize_command(f"PPID {values['pos_kp_milesimas']} {values['pos_kd_milesimas']}")
            normalize_command(f"TOL {values['tolerancia']}")
            if values["ventana"]:
                normalize_command(f"WINDOW {values['pos_min']} {values['pos_max']}")
        return values
    except (ValueError, KeyError):
        return None


class VisibleHistory:
    """Solo la ventana visible va a las curvas; el CSV se guarda por separado."""
    def __init__(self, window=30, maximum=2000):
        self.window = window
        self.items = deque(maxlen=maximum)

    def append(self, elapsed, sample):
        self.items.append((elapsed, sample))
        while len(self.items) > 1 and self.items[0][0] < elapsed - self.window:
            self.items.popleft()

    def curves(self, fields):
        if not self.items:
            return [], {field: [] for field in fields}
        latest = self.items[-1][0]
        times = [elapsed - latest for elapsed, _ in self.items]
        values = {field: [float("nan") if getattr(sample, field) is None
                          else getattr(sample, field) for _, sample in self.items]
                  for field in fields}
        return times, values


def run_panel(port, *, control=False, log=None, window=30, save=None, connection=None,
              counts_per_rev=DEFAULT_COUNTS_PER_REV):
    counts_to_rpm(0, counts_per_rev)
    import tkinter as tk
    from tkinter import ttk
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from serial_link import SerialLink

    root = tk.Tk()
    root.title("RoboClaw + NXP | Velocidad y posición")
    root.geometry("1200x950")
    root.minsize(950, 850)
    link = None
    try:
        link = SerialLink(port, log=log, connection=connection)
    except Exception:
        root.destroy()
        raise
    root.columnconfigure(0, weight=1)
    root.rowconfigure(2, weight=1)
    mode = tk.StringVar(value="Esperando identificación de la NXP…")
    state = tk.StringVar(value="Sin datos de estado")
    measurements = tk.StringVar(value="Esperando telemetría…")
    rpm_measurements = tk.StringVar(value="RPM estimadas del eje de salida: sin datos")
    speed_rpm = tk.StringVar(value="")
    position_feedback = tk.StringVar(value="")
    feedback = tk.StringVar(value="Solo lectura" if not control else "No se arma al abrir. Primero comprobar la lectura.")
    header = ttk.Frame(root, padding=10)
    header.grid(row=0, column=0, sticky="ew")
    ttk.Label(header, textvariable=mode, font=("Segoe UI", 13, "bold")).pack(side="left")
    ttk.Label(header, textvariable=state).pack(side="right")
    readings = ttk.Frame(root, padding=(12, 3))
    readings.grid(row=1, column=0, sticky="ew")
    ttk.Label(readings, textvariable=measurements, wraplength=1100).pack(anchor="w")
    ttk.Label(readings, textvariable=rpm_measurements).pack(anchor="w")
    ttk.Label(readings, textvariable=position_feedback, wraplength=1100).pack(anchor="w")
    body = ttk.Frame(root)
    body.grid(row=2, column=0, sticky="nsew")
    body.columnconfigure(0, weight=1)
    body.rowconfigure(0, weight=1)

    figure = Figure(figsize=(8, 6), dpi=100)
    axes = figure.subplots(3, 1, sharex=True)
    figure.subplots_adjust(left=.12, right=.98, top=.97, bottom=.10, hspace=.18)
    specs = [("speed_cps", 0, "Velocidad medida", "#0077b6", "-"),
             ("reference_cps", 0, "Referencia", "#e76f51", "-"),
             ("target_cps", 0, "Consigna", "#777777", "--"),
             ("duty_pct", 1, "Duty solicitado NXP", "#2a9d8f", "-"),
             ("position_counts", 2, "Posición / encoder", "#6a4c93", "-"),
             ("position_target_counts", 2, "Destino de posición", "#e76f51", "--")]
    lines = {}
    for field, index, label, color, style in specs:
        lines[field], = axes[index].plot([], [], style, color=color, label=label,
                                        linewidth=1.5, animated=True)
    for axis, unit, limits in zip(axes, ("Conteos/s", "Duty (%)", "Conteos"),
                                  ((-550, 550), (-16, 16), (-100, 100))):
        axis.set_ylabel(unit)
        axis.set_ylim(*limits)
        axis.set_xlim(-window, 0)
        axis.grid(alpha=.25)
        axis.legend(loc="upper left", fontsize=8)
    axes[-1].set_xlabel("Tiempo respecto a la última muestra (s)")
    canvas = FigureCanvasTkAgg(figure, master=body)
    canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
    background = None
    dirty = False

    def remember_background(event=None):
        nonlocal background, dirty
        background = canvas.copy_from_bbox(figure.bbox)
        dirty = True

    canvas.mpl_connect("draw_event", remember_background)
    canvas.draw()

    sidebar_host = ttk.Frame(body)
    sidebar_host.grid(row=0, column=1, sticky="ns")
    sidebar_host.columnconfigure(0, weight=1)
    sidebar_host.rowconfigure(1, weight=1)
    sidebar_view = tk.Canvas(sidebar_host, width=255, highlightthickness=0)
    sidebar_view.grid(row=1, column=0, sticky="nsew")
    sidebar_scroll = ttk.Scrollbar(sidebar_host, orient="vertical", command=sidebar_view.yview)
    sidebar_scroll.grid(row=1, column=1, sticky="ns")
    sidebar_view.configure(yscrollcommand=sidebar_scroll.set)
    sidebar = ttk.Frame(sidebar_view, padding=10)
    sidebar_window = sidebar_view.create_window((0, 0), window=sidebar, anchor="nw")
    sidebar.bind("<Configure>", lambda event: sidebar_view.configure(scrollregion=sidebar_view.bbox("all")))
    sidebar_view.bind("<Configure>", lambda event: sidebar_view.itemconfigure(sidebar_window, width=event.width))

    def scroll_controls(event):
        widget = event.widget
        if widget.winfo_class() == "TSpinbox":
            return
        while widget is not None:
            if widget == sidebar_view:
                sidebar_view.yview_scroll(-int(event.delta / 120), "units")
                return "break"
            widget = getattr(widget, "master", None)
    root.bind("<MouseWheel>", scroll_controls)
    variables = {}
    latest_sample = None
    controls = []
    gain_controls = []
    duty_controls = []
    position_controls = []
    position_setup_controls = []
    position_move_button = None
    confirmed = None
    closed = False
    timer_id = None
    outgoing = queue.PriorityQueue(maxsize=32)
    sent = queue.Queue()
    halt_sender = threading.Event()
    sequence = 0

    def sender():
        # La escritura serial puede esperar; los botones no deben esperar con ella.
        while not halt_sender.is_set():
            try:
                _, _, command = outgoing.get(timeout=.05)
            except queue.Empty:
                continue
            if halt_sender.is_set():
                break
            try:
                link.send(command)
                sent.put((True, command))
            except Exception as exc:
                sent.put((False, str(exc)))

    sender_thread = threading.Thread(target=sender, daemon=True)
    sender_thread.start()

    def send(command):
        nonlocal sequence
        try:
            if link.error:
                raise RuntimeError(link.error)
            value = normalize_command(command)
            if value == "STOP":
                # Descartamos órdenes pendientes para no arrancar después de parar.
                while True:
                    try:
                        outgoing.get_nowait()
                    except queue.Empty:
                        break
            sequence += 1
            outgoing.put_nowait((0 if value == "STOP" else 1, sequence, value))
            feedback.set(f"Pendiente: {value}. Esperando respuesta de la NXP.")
        except (ValueError, RuntimeError, queue.Full) as exc:
            feedback.set(f"No enviado: {exc}")

    def field(label, name, default, low, high, step, parent):
        ttk.Label(parent, text=label).pack(anchor="w", pady=(5, 0))
        variables[name] = tk.StringVar(value=default)
        widget = ttk.Spinbox(parent, textvariable=variables[name], from_=low, to=high,
                             increment=step, width=19)
        widget.pack(fill="x")
        return widget

    def apply_gains():
        try:
            send(gain_command([variables[key].get() for key in ("kp", "ki", "kd")]))
        except ValueError as exc:
            feedback.set(str(exc))

    def apply_position_gains():
        try:
            send(gain_command([variables[key].get() for key in ("pos_kp", "pos_kd")], "PPID", 100))
        except ValueError as exc:
            feedback.set(str(exc))

    if control:
        ttk.Label(sidebar, text="Control de M1", font=("Segoe UI", 11, "bold")).pack(anchor="w")
        controls.append(ttk.Button(sidebar, text="Armar", command=lambda: send("ARM")))
        controls[-1].pack(fill="x", pady=4)
        # La parada queda fija y visible aunque se desplace la columna.
        ttk.Button(sidebar_host, text="DETENER / DESARMAR", command=lambda: send("STOP")).grid(
            row=0, column=0, columnspan=2, sticky="ew", padx=10, pady=5)
        controls.append(field("Consigna (conteos/s)", "speed", "100", -500, 500, 10, sidebar))
        ttk.Label(sidebar, textvariable=speed_rpm).pack(anchor="w")
        controls.append(ttk.Button(sidebar, text="Aplicar velocidad",
                                    command=lambda: send("VEL " + variables["speed"].get())))
        controls[-1].pack(fill="x", pady=5)
        bounds = ttk.LabelFrame(sidebar, text="Límites de velocidad", padding=8)
        bounds.pack(fill="x", pady=8)
        controls.append(field("Mínima (conteos/s)", "min", "-500", INT32_MIN, 0, 100, bounds))
        controls.append(field("Máxima (conteos/s)", "max", "500", 0, INT32_MAX, 100, bounds))
        controls.append(ttk.Button(bounds, text="Aplicar límites", command=lambda: send(
            f"LIMITES {variables['min'].get()} {variables['max'].get()}")))
        controls[-1].pack(fill="x", pady=6)
        gains = ttk.LabelFrame(sidebar, text="PID de velocidad NXP", padding=8)
        gains.pack(fill="x", pady=8)
        duty_controls.append(field("Duty máximo (%) · 0 a 100", "duty", "15", 0, 100, 1, gains))
        duty_controls.append(ttk.Button(gains, text="Aplicar duty máximo", command=lambda: send(
            "DUTYMAX " + variables["duty"].get())))
        duty_controls[-1].pack(fill="x", pady=6)
        for name, default in (("kp", "0.020"), ("ki", "0.050"), ("kd", "0.000")):
            gain_controls.append(field(name.upper(), name, default, 0, 1, .001, gains))
        gain_controls.append(ttk.Button(gains, text="Aplicar PID en vivo", command=apply_gains))
        gain_controls[-1].pack(fill="x", pady=6)
        ttk.Label(gains, text="No modifica el PID interno\nde RoboClaw.", wraplength=205).pack(anchor="w")
        positions = ttk.LabelFrame(sidebar, text="Posición NXP · cascada", padding=8)
        positions.pack(fill="x", pady=8)
        position_setup_controls.append(ttk.Button(positions, text="Establecer cero lógico", command=lambda: send("ZERO")))
        position_setup_controls[-1].pack(fill="x", pady=4)
        for label, name, default in (("Recorrido mínimo (conteos)", "pos_min", "-2000"),
                                     ("Recorrido máximo (conteos)", "pos_max", "2000")):
            position_setup_controls.append(field(label, name, default, INT32_MIN, INT32_MAX, 100, positions))
        position_setup_controls.append(ttk.Button(positions, text="Aplicar recorrido", command=lambda: send(
            f"WINDOW {variables['pos_min'].get()} {variables['pos_max'].get()}")))
        position_setup_controls[-1].pack(fill="x", pady=4)
        position_controls.append(field("Destino absoluto (conteos)", "position", "1000", INT32_MIN, INT32_MAX, 100, positions))
        position_move_button = ttk.Button(positions, text="Aplicar posición", command=lambda: send(
            "POS " + variables["position"].get()))
        position_move_button.pack(fill="x", pady=4)
        for label, name, default in (("Kp posición (1/s)", "pos_kp", "0.500"),
                                     ("Kd posición (sin unidad)", "pos_kd", "0.000")):
            position_controls.append(field(label, name, default, 0, 100, .001, positions))
        position_controls.append(ttk.Button(positions, text="Aplicar P/PD posición", command=apply_position_gains))
        position_controls[-1].pack(fill="x", pady=4)
        position_controls.append(field("Tolerancia (conteos)", "tolerance", "10", 1, INT32_MAX, 1, positions))
        position_controls.append(ttk.Button(positions, text="Aplicar tolerancia", command=lambda: send(
            "TOL " + variables["tolerance"].get())))
        position_controls[-1].pack(fill="x", pady=4)
        ttk.Label(positions, text="Cero y recorrido: solo desarmado.\nPOS 0 regresa al cero; STOP detiene.", wraplength=215).pack(anchor="w")
        ttk.Button(sidebar, text="Consultar configuración", command=lambda: send("INFO")).pack(fill="x", pady=8)
        for widget in controls + gain_controls + duty_controls + position_controls + position_setup_controls + [position_move_button]:
            widget.state(["disabled"])
    else:
        ttk.Label(sidebar, text="Solo lectura\nNo enviamos comandos.").pack(anchor="w")
    conversion = ttk.LabelFrame(sidebar, text="Conversión a RPM", padding=8)
    conversion.pack(fill="x", pady=6)
    field("Conteos por vuelta del eje", "cpr", f"{counts_per_rev:g}", 1, INT32_MAX, 1, conversion)
    ttk.Label(conversion, text="Solo cambia la visualización.\nLas órdenes siguen en conteos/s.",
              wraplength=215).pack(anchor="w", pady=(4, 0))

    def update_rpm(*_):
        try:
            factor = float(variables["cpr"].get())
            counts_to_rpm(0, factor)
            def rpm(value):
                result = counts_to_rpm(value, factor)
                return "—" if result is None else f"{result:.2f}"
            fresh = latest_sample is not None and time.monotonic() - last_rx <= 2
            measured = latest_sample.speed_cps if fresh else None
            target = latest_sample.target_cps if fresh else None
            rpm_measurements.set(f"RPM estimadas del eje de salida: medida {rpm(measured)} | "
                                 f"consigna activa {rpm(target)} | {factor:g} conteos/vuelta")
            if "speed" in variables:
                try:
                    speed_rpm.set(f"Consigna escrita: ≈ {rpm(int(variables['speed'].get()))} RPM")
                except ValueError:
                    speed_rpm.set("Consigna escrita: — RPM")
        except (ValueError, OverflowError):
            rpm_measurements.set("RPM no disponibles: introducir conteos/vuelta positivos y finitos.")
            speed_rpm.set("Conversión a RPM no disponible")

    variables["cpr"].trace_add("write", update_rpm)
    if "speed" in variables:
        variables["speed"].trace_add("write", update_rpm)
    ttk.Label(sidebar, text="Sin vencimiento por inactividad.\nSTOP/SW2 son paradas por software.\nMantener el paro físico accesible.",
              wraplength=225).pack(anchor="w", pady=10)
    ttk.Label(root, textvariable=feedback, padding=10, wraplength=1150).grid(row=3, column=0, sticky="ew")
    output = tk.Text(root, height=5, state="disabled", font=("Consolas", 9))
    output.grid(row=4, column=0, sticky="ew", padx=10, pady=(0, 8))

    def add_message(line):
        output.configure(state="normal")
        output.insert("end", line + "\n")
        if int(output.index("end-1c").split(".")[0]) > 120:
            output.delete("1.0", "21.0")
        output.see("end")
        output.configure(state="disabled")

    def configure(values):
        nonlocal confirmed
        confirmed = values
        stage = values["etapa"]
        mode.set({1: "Versión · solo lectura", 2: "Telemetría · solo lectura",
                  3: "Velocidad · PID interno de RoboClaw", 4: "Velocidad · PID propio NXP",
                  5: "Posición · P/PD y PID de velocidad NXP"}[stage])
        state.set("ARMADO" if values["armado"] else "DESARMADO")
        if control:
            for widget in controls:
                widget.state(["!disabled"] if stage in (3, 4, 5) and not link.error else ["disabled"])
            for widget in controls[1:3]:
                widget.state(["!disabled"] if stage in (3, 4) and not link.error else ["disabled"])
            for widget in gain_controls:
                widget.state(["!disabled"] if stage in (4, 5) and not link.error else ["disabled"])
            for widget in duty_controls:
                widget.state(["!disabled"] if stage in (4, 5) and "duty_maximo" in values
                             and not link.error else ["disabled"])
            if "duty_maximo" in values:
                variables["duty"].set(str(values["duty_maximo"]))
            if stage in (4, 5) and "duty_maximo" in values:
                extent = max(1, values["duty_maximo"] * 1.07)
                axes[1].set_ylim(-extent, extent)
                canvas.draw()
            variables["min"].set(str(values["minimo"]))
            variables["max"].set(str(values["maximo"]))
            for key in ("kp", "ki", "kd"):
                variables[key].set(f"{values[key + '_milesimas'] / 1000:.3f}")
            controls[1].configure(from_=values["minimo"], to=values["maximo"])
            for widget in position_controls + position_setup_controls + [position_move_button]:
                widget.state(["!disabled"] if stage == 5 and not link.error else ["disabled"])
            if stage == 5:
                if values["ventana"]:
                    for key in ("pos_min", "pos_max"):
                        variables[key].set(str(values[key]))
                    position_controls[0].configure(from_=values["pos_min"] + 1, to=values["pos_max"] - 1)
                variables["pos_kp"].set(f"{values['pos_kp_milesimas'] / 1000:.3f}")
                variables["pos_kd"].set(f"{values['pos_kd_milesimas'] / 1000:.3f}")
                variables["tolerance"].set(str(values["tolerancia"]))
                update_position_buttons(values["armado"])
            else:
                position_feedback.set("")
        lines["position_target_counts"].set_visible(stage == 5)
        feedback.set("Configuración confirmada por la NXP. Los campos no se aplican hasta pulsar Aplicar.")

    def update_position_buttons(armed):
        if not control or not confirmed or confirmed["etapa"] != 5:
            return
        ready = confirmed["cero_definido"] and confirmed["ventana"] and confirmed.get("duty_maximo", 0) > 0
        for widget in position_setup_controls:
            widget.state(["!disabled"] if not armed and not link.error else ["disabled"])
        position_move_button.state(["!disabled"] if armed and ready and not link.error else ["disabled"])
        controls[0].state(["!disabled"] if ready and not armed and not link.error else ["disabled"])

    history = VisibleHistory(window)
    last_rx = time.monotonic()
    last_draw = 0.0
    last_scale = 0.0
    dirty = False
    fault_handled = False

    def redraw(now):
        nonlocal dirty, last_draw, last_scale
        times, values = history.curves(lines)
        for name, line in lines.items():
            line.set_data(times, values[name])
        changed_scale = False
        if now - last_scale >= 1:
            last_scale = now
            for axis, names in ((axes[0], ("speed_cps", "reference_cps", "target_cps")),
                                (axes[1], ("duty_pct",)),
                                (axes[2], ("position_counts", "position_target_counts"))):
                finite = [v for name in names for v in values[name] if math.isfinite(v)]
                if finite:
                    low, high = axis.get_ylim()
                    if min(finite) < low or max(finite) > high:
                        span = max(max(finite) - min(finite), 5 if axis == axes[1] else 100)
                        axis.set_ylim(min(finite) - span * .15, max(finite) + span * .15)
                        changed_scale = True
        if changed_scale or background is None:
            canvas.draw()
        canvas.restore_region(background)
        for line in lines.values():
            line.axes.draw_artist(line)
        canvas.blit(figure.bbox)
        dirty = False
        last_draw = now

    def tick():
        nonlocal last_rx, dirty, fault_handled, timer_id, latest_sample
        if closed:
            return
        now = time.monotonic()
        for _ in range(200):
            try:
                event = link.events.get_nowait()
            except queue.Empty:
                break
            sample = parse_line(event.line)
            if sample:
                history.append(event.elapsed, sample)
                last_rx = now
                latest_sample = sample
                dirty = True
                def shown(value):
                    return "—" if value is None else str(value)
                measurements.set(f"Velocidad: {shown(sample.speed_cps)} conteos/s | Consigna: {shown(sample.target_cps)} | "
                                 f"Duty NXP: {shown(sample.duty_pct)}% (máx. {shown(sample.duty_max_pct)}%) | "
                                 f"Voltaje: {shown(sample.battery_v)} V | "
                                 f"Intervalo PID: {shown(sample.dt_us)} µs")
                update_rpm()
                if sample.encoder_raw_counts is not None or (confirmed and confirmed["etapa"] == 5):
                    position_feedback.set(f"Posición lógica: {shown(sample.position_counts)} conteos | "
                        f"Destino: {shown(sample.position_target_counts)} | Error: {shown(sample.position_error_counts)} | "
                        f"En posición: {'—' if sample.position_reached is None else 'sí' if sample.position_reached else 'no'}")
                if sample.armed is not None:
                    state.set("ARMADO" if sample.armed else "DESARMADO")
                    update_position_buttons(sample.armed)
            elif event.line:
                add_message(event.line)
                config = parse_config(event.line)
                if config:
                    configure(config)
                elif event.line.startswith("DESARMADO "):
                    state.set("DESARMADO")
                    update_position_buttons(0)
                    feedback.set(event.line)
                else:
                    feedback.set(event.line)
        for _ in range(32):
            try:
                ok, value = sent.get_nowait()
            except queue.Empty:
                break
            add_message(f"> {value}" if ok else f"No enviado: {value}")
        if link.error:
            feedback.set(f"FALLO SERIAL: {link.error}. Comprobar el eje y usar paro físico.")
            if not fault_handled:
                fault_handled = True
                for widget in controls + gain_controls + duty_controls + position_controls + position_setup_controls:
                    widget.state(["disabled"])
                if position_move_button is not None:
                    position_move_button.state(["disabled"])
                if control:
                    # El intento de parada no equivale a entrega garantizada.
                    try:
                        sequence_stop = (0, -1, "STOP")
                        outgoing.put_nowait(sequence_stop)
                    except queue.Full:
                        pass
        elif now - last_rx > 2:
            measurements.set("Sin telemetría reciente. La orden puede seguir activa; revisar conexión y Resume.")
            if confirmed and confirmed["etapa"] == 5:
                position_feedback.set("Posición y llegada sin confirmar: falta telemetría reciente.")
            update_rpm()
        if control and confirmed is None and now - link.started > 3:
            mode.set("Falta CONFIG: cargar el firmware actualizado y consultar configuración")
        if dirty and now - last_draw >= .05:
            redraw(now)
        timer_id = root.after(20, tick)

    def close():
        nonlocal closed
        if closed:
            return
        closed = True
        if timer_id is not None:
            root.after_cancel(timer_id)
        halt_sender.set()
        sender_thread.join(timeout=.3)
        warning = link.close(stop=control)
        if control:
            print(warning or "STOP enviado al cerrar, sin confirmar eje detenido.")
        try:
            if save:
                # Para exportar incluimos las curvas, que en vivo se dibujan aparte.
                from plot_live import save_figure
                for line in lines.values():
                    line.set_animated(False)
                save_figure(figure, save)
        finally:
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)
    if control:
        send("INFO")  # Solo consulta; no arma ni ordena movimiento.
    update_rpm()
    tick()
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        close()


def main():
    parser = Parser(description=__doc__)
    parser.add_argument("port", help="COM de OpenSDA, por ejemplo COM8")
    parser.add_argument("--solo-lectura", action="store_true", help="No enviar ni INFO ni STOP")
    parser.add_argument("--log", help="CSV nuevo para este ensayo")
    parser.add_argument("--window", type=float, default=30, help="Ventana visible en segundos")
    parser.add_argument("--conteos-vuelta", type=float, default=DEFAULT_COUNTS_PER_REV,
                        help="Conteos por vuelta del eje de salida para mostrar RPM (3332 por defecto)")
    args = parser.parse_args()
    if not math.isfinite(args.window) or not 0 < args.window <= 3600:
        parser.error("La ventana debe estar entre 0 y 3600 segundos.")
    try:
        counts_to_rpm(0, args.conteos_vuelta)
    except ValueError as exc:
        parser.error(str(exc))
    run_panel(args.port, control=not args.solo_lectura, log=args.log, window=args.window,
              counts_per_rev=args.conteos_vuelta)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ImportError, ValueError) as exc:
        raise SystemExit(str(exc))
