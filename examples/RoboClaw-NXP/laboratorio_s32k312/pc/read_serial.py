"""Monitor de lectura: mostramos los datos y, si se pide, guardamos el CSV."""
from cli import Parser
import queue
import time
from serial_link import SerialLink, list_ports


def main():
    parser = Parser(description=__doc__)
    parser.add_argument("port", nargs="?", help="COM OpenSDA NXP, por ejemplo COM8")
    parser.add_argument("--list", action="store_true", help="Listar puertos sin abrirlos")
    parser.add_argument("--log", help="CSV nuevo, sin sobrescribir")
    parser.add_argument("--seconds", type=float, help="Duracion de lectura; NO para el motor")
    args = parser.parse_args()
    if args.list:
        list_ports()
        return
    if not args.port:
        parser.error("Indicar puerto o --list.")
    if args.seconds is not None and not 0 < args.seconds <= 86400:
        parser.error("--seconds debe estar entre 0 y 86400.")
    link = SerialLink(args.port, log=args.log)
    print("Solo lectura: NO envia ARM, STOP ni ningun comando. Ctrl+C termina.")
    try:
        while args.seconds is None or time.monotonic() - link.started < args.seconds:
            if link.error:
                raise RuntimeError(link.error)
            try:
                event = link.events.get(timeout=0.1)
                print(event.line)
            except queue.Empty:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        if link.dropped:
            print(f"Lineas no mostradas: {link.dropped}; CSV no depende de la pantalla.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ImportError) as exc:
        raise SystemExit(str(exc))
