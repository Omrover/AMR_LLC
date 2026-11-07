"""Consola para mandar comandos a la NXP, sin armar el motor al abrir."""
from cli import Parser
import queue
import threading
import time
from serial_link import SerialLink
from telemetry import normalize_command


def main():
    parser = Parser(description=__doc__)
    parser.add_argument("port", help="COM de OpenSDA NXP, NO USB RoboClaw")
    parser.add_argument("--log", help="CSV nuevo")
    args = parser.parse_args()
    link = SerialLink(args.port, log=args.log)
    commands = queue.Queue(maxsize=128)

    def keyboard():
        while True:
            try:
                line = input()
            except EOFError:
                commands.put("QUIT")
                return
            commands.put(line)
            if line.strip().upper() == "QUIT":
                return

    print("INFO | HELP | ARM | VEL 100 | LIMITES -500 500 | PID 20 50 0 | DUTYMAX 30 | STOP | QUIT")
    print("Posición NXP: ZERO | WINDOW -2000 2000 | POS 1000 | PPID 500 0 | TOL 10")
    print("No arma al abrir. Sin vencimiento por inactividad; QUIT/Ctrl+C intenta STOP.")
    print("Enviar STOP no confirma el acuse ni la parada del eje. Mantener una parada física accesible.")
    threading.Thread(target=keyboard, daemon=True).start()
    try:
        while True:
            if link.error:
                raise RuntimeError(link.error)
            for _ in range(100):
                try:
                    print(link.events.get_nowait().line)
                except queue.Empty:
                    break
            try:
                text = commands.get_nowait()
            except queue.Empty:
                time.sleep(0.01)
                continue
            if text.strip().upper() == "QUIT":
                break
            try:
                command = normalize_command(text)
                if command:
                    link.send(command)
                    print(f"> {command}")
            except ValueError as exc:
                print(f"No enviado: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        warning = link.close(stop=True)
        print(warning or "STOP enviado al salir, sin confirmar el acuse. Verificar que el motor esté detenido.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ImportError) as exc:
        raise SystemExit(str(exc))
