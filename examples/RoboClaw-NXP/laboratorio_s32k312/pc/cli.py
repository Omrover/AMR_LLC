"""La ayuda de las herramientas también se muestra en español."""
import argparse


class SpanishFormatter(argparse.HelpFormatter):
    def add_usage(self, usage, actions, groups, prefix=None):
        super().add_usage(usage, actions, groups, prefix or "Uso: ")


class Parser(argparse.ArgumentParser):
    def __init__(self, **kwargs):
        super().__init__(add_help=False, formatter_class=SpanishFormatter, **kwargs)
        self._positionals.title = "Argumentos"
        self._optionals.title = "Opciones"
        self.add_argument("-h", "--help", action="help", help="Mostrar esta ayuda y salir")
