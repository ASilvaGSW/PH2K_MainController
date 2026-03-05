# Estrategia para acciones en paralelo con CAN bloqueante

## Situación actual

- **oneCycle()** ejecuta muchas acciones en serie; cada una llama a dispositivos vía CAN.
- **send_message()** en `canbus.py` es **bloqueante**: envía, hace `read_message()` y espera la confirmación (respuesta con `can_id + 0x400`) antes de devolver.
- El bus CAN es **uno solo**: si dos hilos llaman `send_message` a la vez, uno hace flush y envía y el otro puede “robar” la respuesta o bloquear el canal.

Por eso no se pueden lanzar varias acciones “en paralelo” solo con threads usando el mismo `send_message` bloqueante: el bus y la espera de confirmación están acoplados a una sola llamada.

---

## Estrategia recomendada: dispatcher de respuestas (un lector, muchos “waiters”)

Objetivo: **desacoplar “enviar” de “esperar confirmación”** y repartir las respuestas por `(can_id, function_id)` para que varias operaciones puedan estar “en vuelo” a la vez.

### Idea

1. **Un único hilo lector** que solo lee del canal CAN y, cuando llega un frame de respuesta (`can_id` en rango de respuestas, p. ej. `request_id + 0x400`), lo entrega al “waiter” correcto según `can_id` (y opcionalmente `function_id`).
2. **Envío sin esperar**: se añade algo tipo `send_async(can_id, data)` que:
   - (Opcional) no hace `flush_buffer()` para no tirar respuestas de otros dispositivos (o se hace un flush selectivo).
   - Escribe el mensaje en el bus (`channel.write`).
   - Registra un “waiter” (por ejemplo un `threading.Event` + resultado) para la respuesta esperada `(can_id + 0x400, data[0])`.
   - Devuelve de inmediato un objeto “futuro” (o el Event + resultado) para que el llamador espere cuando quiera.
3. El **lector** cuando recibe un mensaje con ese `can_id` (y `function_id` si se usa) marca el waiter como completado y guarda éxito/error.
4. En **oneCycle** (o en funciones auxiliares), se identifican bloques que pueden ejecutarse en paralelo (p. ej. mover insertion_jig y hose_puller a la vez). Esos se lanzan con `send_async` (o wrappers por dispositivo) y luego se hace “wait” de todos los futuros antes de continuar la secuencia.

Así los comandos CAN siguen siendo “enviar + confirmación”, pero la confirmación se espera de forma desacoplada y por operación, permitiendo paralelismo real cuando el proceso lo permita.

### Ventajas

- No se cambia el protocolo CAN (mismo formato de request/response).
- Un solo punto de lectura evita condiciones de carrera en el buffer.
- Se puede seguir usando la misma API bloqueante (`send_message`) donde no interese paralelizar, y usar la nueva API async solo donde sí.

### Consideraciones

- **Orden de respuestas**: si envías A y B casi a la vez, puede llegar antes la respuesta de B que la de A; el dispatcher por `(can_id, function_id)` entrega cada respuesta al waiter correcto, así que el orden no importa.
- **Mismo dispositivo**: si dos comandos al mismo `can_id` pueden estar en vuelo, hay que distinguir por `function_id` (o secuencia) para no asignar la respuesta al waiter equivocado.
- **flush_buffer()**: con múltiples comandos en vuelo no se puede hacer un flush agresivo antes de cada envío; o se elimina en modo async o se hace un flush solo al inicio de una “ronda” de envíos paralelos.

---

## Alternativa más simple (sin tocar la capa CAN)

Si no quieres implementar el dispatcher aún:

1. **Identificar secuencias independientes** en `oneCycle`: por ejemplo, “mover insertion_jig a posición X” y “mover hose_puller a Y” son lógicamente independientes si no comparten pieza ni orden crítico.
2. **No paralelizar el bus**: seguir con un solo hilo que hace `send_message` en serie.
3. **Paralelizar solo lo que no sea CAN**: por ejemplo cálculos, preparación de datos, o I/O (red, disco) en otros hilos, mientras el hilo principal hace la secuencia CAN. Así reduces tiempo total solo en la parte no-CAN.

Esta opción no da paralelismo real en los comandos CAN, pero no requiere cambios en `canbus.py`.

---

## Dónde paralelizar en oneCycle (ejemplos)

Solo tiene sentido paralelizar cuando:

- Dos dispositivos **distintos** hacen movimientos (o acciones) que no dependen uno del otro en esa fase del ciclo.
- Ejemplo: después de “Insertion Jig Home”, podrías tener un bloque donde `insertion_jig.move_z_axis(...)` y `hose_puller.move_y_actuator(...)` (u otro par) se envían en paralelo y luego se hace “wait all” antes de seguir.

Hay que revisar **dependencias mecánicas y de proceso**: muchos pasos deben seguir en orden (cerrar gripper antes de mover, etc.). Los candidatos a paralelo suelen ser movimientos de ejes de dispositivos diferentes que no se cruzan.

---

## Resumen

- **Estrategia sólida para paralelismo real con CAN**: dispatcher de respuestas (un lector + `send_async` + waiters por `can_id`/`function_id`) en la capa CAN, y en `oneCycle` lanzar en paralelo solo los bloques independientes y hacer wait de todos antes de continuar.
- **Estrategia sin tocar CAN**: mantener todo secuencial en el bus y paralelizar solo lógica no-CAN (cálculos, I/O, etc.).

Si quieres, el siguiente paso puede ser esbozar la API concreta (`send_async`, estructura del waiter, y cómo iniciar el hilo lector) en tu `canbus.py` y un mini-ejemplo de uso en una función tipo oneCycle.
