# stm_pwm_min_test: prueba de PWM mínimo de los motores

Proyecto STM32CubeIDE derivado de
`omnidirectional-robot-with-vision-ai-stm32/firmware/stm_unified`. Usa el
mismo hardware y la misma arquitectura (STM32F446RE, FreeRTOS, micro-ROS por
USART2, los mismos timers de encoder y PWM, los mismos enables y la misma
convención de signos). En lugar del lazo PID y la odometría, corre una prueba
que mide **cuál es el PWM mínimo con el que cada motor se mueve**.

## Por qué se miden dos valores

Un motor con reductora tiene dos umbrales distintos:

| Valor | Qué es | Cuándo importa |
|---|---|---|
| **pwm_start** | PWM mínimo para *arrancar* desde el reposo (fricción estática) | Al salir desde parado |
| **pwm_stop** | PWM mínimo para *seguir girando* cuando ya está en movimiento (fricción dinámica) | Cuando el control de trayectoria baja la velocidad al acercarse al objetivo |

`pwm_stop` casi siempre es bastante menor que `pwm_start`. Si solo se sube el
PWM hasta que la rueda se mueve, se obtiene `pwm_start`, que sobreestima lo
que hace falta para *mantener* el movimiento. Por eso la prueba sube el PWM,
y cuando la rueda arranca, lo vuelve a bajar hasta que la rueda se detiene.

## Método (Core/Src/pwm_test.c)

Para cada rueda (o grupo de ruedas) y cada sentido de giro:

1. **REST**: PWM en 0 durante 1 s.
2. **RAMP_UP**: el PWM sube por escalones de 300 ms. Los primeros 100 ms de
   cada escalón se descartan (transitorio) y en los 200 ms restantes se
   cuentan pulsos de encoder. Se considera que la rueda se mueve si hay
   **≥ 4 pulsos en el sentido pedido (≈ 0.33 rad/s) durante 2 escalones
   seguidos**. `pwm_start` es el primer escalón de esa racha.
3. **BOOST**: 0.5 s a `pwm_start + 40` para asegurar que la rueda está en
   régimen dinámico.
4. **RAMP_DOWN**: el PWM baja por escalones hasta que la rueda deja de
   cumplir el criterio de movimiento durante 2 escalones seguidos.
   `pwm_stop` es el último PWM con el que la rueda todavía se movía.

Los pulsos se cuentan en una ventana de 200 ms y no se usa la velocidad
instantánea a 10 ms del PID. Con 384.5 pulsos por vuelta, 1 pulso en 10 ms
equivale a 1.63 rad/s, así que a baja velocidad esa medición es casi solo
ruido de cuantización. En 200 ms la resolución es de ~0.08 rad/s.

Para no tardar varios minutos por rueda, cada combinación hace primero un
**barrido grueso** (trial 0, escalones de 10 cuentas desde 0). Después hace
`trials` **barridos finos** (escalones de 2 cuentas) que empiezan 40 cuentas
por debajo de lo encontrado en el barrido grueso. Así la resolución final es
de 2 cuentas (0.2 % de duty, ≈ 0.024 V) y, en el modo de grupo, el robot se
desplaza poco.

La lógica se validó en PC compilando `pwm_test.c` contra un modelo de motor
con umbrales conocidos: encontró `pwm_start` exacto y `pwm_stop` con un error
de 0 a 2 cuentas, con las 4 ruedas y en ambos sentidos.

## Modos

- **single** (`mode = 1`): una rueda a la vez, con las demás en 0. Da la
  característica de cada motor. Con el robot **levantado** mide solo
  motor + reductora. Con el robot **en el piso**, las demás ruedas frenan,
  así que el valor es pesimista.
- **group** (`mode = 2`): las 4 ruedas a la vez con el mismo PWM y siguiendo
  un patrón (`forward` `[+,+,+,+]`, `lateral` `[+,-,-,+]`, `rotation`
  `[+,-,+,-]`, en orden FR, FL, BR, BL). Con el robot **en el piso** es la
  condición real de trabajo. Se reporta el umbral de cada rueda. El del robot
  es el de la rueda más exigente. En mecanum, `lateral` suele necesitar más
  PWM por la fricción de los rodillos.

**Recomendación:** corre `group` con el robot en el piso, en el patrón que más
uses (y también en `lateral`), y toma esos valores como definitivos. Usa
`single` para comparar motores entre sí o para diagnosticar una rueda que se
comporta distinto.

## Interfaz micro-ROS

| Tópico | Tipo | Contenido |
|---|---|---|
| `/pwm_test/cmd` | `Int32MultiArray` | `[mode, target, dir, trials, pwm_limit]` |
| `/pwm_test/status` | `Float64MultiArray` | `[state, mode, target, dir, trial, pwm, vel_fr, vel_fl, vel_br, vel_bl]` (uno por escalón, y un latido cada 1 s en reposo) |
| `/pwm_test/result` | `Float64MultiArray` | `[mode, pattern, wheel, dir, trial, pwm_start, pwm_stop, vel_start, vel_stop, flags]` (uno por rueda e intento) |

- `mode`: 0 = abortar, 1 = single, 2 = group.
- `target`: en single, máscara de ruedas (1 = FR, 2 = FL, 4 = BR, 8 = BL,
  15 = todas). En group, el patrón (0 = forward, 1 = lateral, 2 = rotation).
- `dir`: +1, -1 o 0 (ambos sentidos). `trials`: 1..10 (por defecto 3).
  `pwm_limit`: tope de seguridad en cuentas (por defecto 600 de 999).
- `flags`: bit0 = `pwm_start` válido, bit1 = `pwm_stop` válido, bit2 = la
  rueda giró en sentido contrario al pedido (revisar cableado o signo).
- `state`: 0 IDLE, 1 REST, 2 RAMP_UP, 3 BOOST, 4 RAMP_DOWN, 5 DONE,
  6 ABORTED, 7 REJECTED.

El LED de la placa (PA5) queda encendido mientras la prueba corre y parpadea
si falla la inicialización de micro-ROS (igual que en `stm_unified`).

## Cómo correrla

1. En STM32CubeIDE: *File → Import → Existing Projects into Workspace* y
   selecciona esta carpeta. Compila y flashea como siempre. El pre-build con
   docker es el mismo que en `stm_unified`, y la librería micro-ROS ya viene
   compilada en `micro_ros_stm32cubemx_utils/microros_static_library_ide`.
2. Levanta el agente:
   `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200`
   y resetea la placa.
3. Corre el script:

```bash
cd scripts
python3 pwm_min_test.py --mode group --pattern forward     # robot en el piso
python3 pwm_min_test.py --mode group --pattern lateral
python3 pwm_min_test.py --mode single                      # cada rueda, ambos sentidos
python3 pwm_min_test.py --mode single --wheels BL --dir pos --trials 5
```

Ctrl+C aborta la prueba y deja los motores en 0. El script guarda dos CSV en
`scripts/resultados_pwm/`: `pasos_*.csv`, con PWM y velocidad de cada
escalón (sirve para graficar la curva PWM → velocidad cerca de la zona
muerta), y `resultados_*.csv`. Al final imprime una tabla con el máximo y la
media de `pwm_start` y `pwm_stop` por rueda y sentido, la velocidad mínima
sostenible y un arreglo `U_MIN_WHEEL[4]` en voltios listo para pegar.

Duración aproximada: `single` con las 4 ruedas, ambos sentidos y 3 trials
tarda unos 7 min. `group`, ambos sentidos y 3 trials, unos 3 min.

También se puede lanzar sin el script:

```bash
ros2 topic pub --once /pwm_test/cmd std_msgs/msg/Int32MultiArray "{data: [2, 0, 0, 3, 600]}"
ros2 topic echo /pwm_test/result
```

## Cómo usar el resultado en `stm_unified`

En `control_PID.c`, `u` está en voltios y se convierte a PWM como
`pwm = |u| / 12 * 999`. El umbral en voltios es `U_MIN = pwm_stop * 12 / 999`.
Hay dos maneras de usarlo, y conviene combinarlas:

1. **Compensar la zona muerta en el firmware.** En `compute_wheel_cmd`,
   cuando la referencia no es cero (ya está el `REF_DEADBAND`), no dejes que
   el comando final quede por debajo del mínimo:

   ```c
   if (w->cmd > 0.0f && w->cmd <  U_MIN_WHEEL[i]) w->cmd =  U_MIN_WHEEL[i];
   if (w->cmd < 0.0f && w->cmd > -U_MIN_WHEEL[i]) w->cmd = -U_MIN_WHEEL[i];
   ```

   Así, cualquier referencia pequeña produce al menos el PWM que sostiene el
   giro, en vez de un PWM que no mueve nada mientras el integrador (KI = 0.047)
   tarda en acumular. Si prefieres una respuesta lineal, puedes remapear
   `u_eff = sign(u) * (U_MIN + |u| * (12 - U_MIN) / 12)`. En ese caso conviene
   volver a caracterizar la tabla de feedforward, porque la tabla actual ya
   incluye la zona muerta de forma implícita.

2. **Limitar la velocidad mínima en el control de trayectoria.** Por debajo
   de la "velocidad mínima sostenible" que imprime el script, la rueda no
   puede girar de forma estable, pongas el PWM que pongas. En
   `trajectory_controller.py` / `trajectory_follower.py`, en vez de dejar que
   la referencia tienda a 0, satúrala a esa velocidad mínima hasta entrar en
   la tolerancia de posición, y ahí manda 0. Esto evita que el robot se quede
   "arrastrándose" sin llegar.

## Notas

- Se eliminaron `control_PID`, `odometry_omni` e `imu_simple`. I2C1 se sigue
  inicializando (como en `stm_unified`) pero no se usa.
- **No regeneres el código desde el `.ioc`**: el `.ioc` solo declara una
  `defaultTask` genérica (igual que en `stm_unified`) y CubeMX reemplazaría las
  tareas escritas a mano en `freertos.c`.
- Los parámetros de la prueba (duración del escalón, pulsos mínimos, tamaño
  del escalón, BOOST, etc.) están al principio de `Core/Src/pwm_test.c`.
- La prueba mueve el robot en el modo `group`. Déjale espacio libre, sobre
  todo en `lateral` y `rotation`.
