# Laboratorio de velocidad y posición: S32K312 + RoboClaw


### Comunicación NXP–RoboClaw

| Señal | Conexión |
|---|---|
| TX NXP | PTD2, J11-5 → señal S1 de RoboClaw |
| RX NXP | PTD3, J11-7 ← señal S2 de RoboClaw |
| Tierra | GND NXP ↔ GND RoboClaw |

Conectar la computadora al USB OpenSDA de la NXP. Este puerto proporciona la consola a **115200 baud**. Elegir el puerto COM correspondiente; los comandos de esta guía usan COM8 como ejemplo.

Alimentar RoboClaw con su fuente de potencia. No unir las salidas +5V de RoboClaw y de la placa. La E/S de esta placa utiliza 5V por defecto y S2 transmite a 3.3V; emplear una adaptación de nivel apropiada en recepción. No colocar un pull-up a 5V sobre S2.

### Configuración en Basicmicro Motion Studio

1. Seleccionar Packet Serial.
2. Establecer dirección **128** y velocidad **38400 baud**, formato 8N1.
3. Seleccionar comunicación de una unidad y timeout de **3 segundos**.
4. Verificar la lectura del encoder M1 y el sentido de giro.
5. Para el control interno, configurar las ganancias de velocidad y QPPS del motor.
6. Guardar mediante Write Settings y comprobar que los ajustes permanecen después de reiniciar RoboClaw.

El código de aplicación controla únicamente M1.


| Archivo en nxp/ejemplos | Perfil | Función |
|---|---|---|
| 01_version.c | version | Consulta la versión de RoboClaw para comprobar TX/RX |
| 02_lectura.c | telemetry | Lee posición, velocidad y voltaje |
| 03_velocidad_roboclaw.c | internal | Envía la consigna al PID de velocidad interno de RoboClaw |
| 04_pid_nxp.c | pid | Calcula el PID en la NXP y envía el duty a RoboClaw |
| 05_posicion_nxp.c | position | Calcula una velocidad a partir del error de posición y utiliza el PID NXP para generar duty |

El modo interno utiliza SpeedAccelM1. Los modos de control NXP utilizan ReadSpeedM1 para la velocidad y DutyM1 para la salida. El modo position consulta también ReadEncM1: ambos lazos se calculan en la NXP, sin usar el PID interno de posición de RoboClaw. Las protecciones de RoboClaw permanecen activas.

Cada archivo de ejemplo selecciona una etapa e incluye el código común nxp/src/main.c. **Compilar una sola entrada**, no todos los ejemplos simultáneamente.

D3 indica el estado de las lecturas: verde para datos válidos y rojo para un fallo. SW2 solicita la parada por software.

LAB_STAGE tiene LAB_TELEMETRY como valor predeterminado. Confirmar en la consola el mensaje PERFIL de la etapa cargada. El panel de control requiere que el firmware responda CONFIG protocolo=2.

Para seleccionar velocidad con PID propio, usar en `lab_profile.h`:

```c
#ifndef LAB_STAGE
#define LAB_STAGE LAB_NXP_PID
#endif
```

Para seleccionar posición, sustituir `LAB_NXP_PID` por `LAB_NXP_POSITION`.
Compilar y cargar nuevamente después de cambiar el perfil; abrir el panel no
cambia el programa de la NXP. En la consola, `INFO` debe mostrar `etapa=4` para
velocidad propia o `etapa=5` para posición.

**Detener el motor antes de programar una etapa de lectura.** Esas etapas no envían órdenes de parada.


### Panel de control

```powershell
python laboratorio_s32k312/pc/panel.py COM8 --log ensayo_01.csv
```

El panel identifica la etapa cargada y muestra velocidad medida, consigna, referencia, duty solicitado y posición del encoder. En position muestra además destino, error de posición y el indicador En posición. No arma el motor al abrir.

Los campos de velocidad, límites y ganancias admiten escritura e incrementos con flechas. Pulsar el botón Aplicar correspondiente para enviar los valores. La respuesta de la NXP confirma la aceptación o indica el motivo del rechazo.

En la etapa internal, el panel permite cambiar velocidad y límites. En pid permite además ajustar Kp, Ki, Kd y duty máximo durante la marcha. En position la consigna de velocidad se calcula automáticamente; se habilitan los campos del grupo Posición NXP · cascada. Desplazar la columna lateral para ver todos los ajustes; DETENER / DESARMAR permanece visible.

Para observar sin enviar comandos:

```powershell
python laboratorio_s32k312/pc/panel.py COM8 --solo-lectura
```

