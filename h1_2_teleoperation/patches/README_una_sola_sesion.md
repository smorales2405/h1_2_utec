# Parche: una sola sesión manda la imagen

`televuer_una_sola_sesion.patch` — se aplica al **submódulo** `televuer`, no a
`xr_teleoperate`:

```bash
cd xr_teleoperate/teleop/televuer
git apply ../../../patches/televuer_una_sola_sesion.patch
```

## El fallo

Vuer arranca un handler **por cada websocket**, y cada handler abre su propio
bucle de imagen. Si el navegador del visor tiene varias pestañas con la página
—o si reconecta sin cerrar la anterior— se acumulan bucles publicando JPEG a la
vez contra el mismo dispositivo.

Con **siete sesiones simultáneas**, medido el 2026-09-10 en el robot real:

| | |
|---|---|
| cola de envío del socket | hasta **931 kB** |
| retransmisiones | **0** |
| paquetes perdidos | **0** |
| pérdida de ping | **0** durante el fallo |

Cero retransmisiones y cero pérdidas con casi un mega encolado significa una
cosa concreta: **la red no falla, el receptor no lee**. El visor no da abasto
decodificando siete flujos, cierra su ventana de recepción y el servidor se
queda con todo atascado en la cola.

Y entonces se realimenta. Al no recibir nada, el vigilante de latido del
cliente —`react-use-websocket`, `timeout: 60000`— hace:

```js
console.warn("Heartbeat timed out, closing connection, ...");
ws.close();
```

Un `close()` sin código, que en el servidor se ve como `close_code=0` con
`exception=None`. El cliente reconecta… y **suma otra sesión**.

## El síntoma que se ve

La imagen llega bien y de pronto se congela; en la terminal, ciclos de
`websocket is connected` / `websocket is now disconnected` y, detrás,
`AssertionError: Websocket session is missing.` — que es **consecuencia**: el
bucle de imagen de una sesión ya cerrada sigue intentando escribir en ella.

## Lo que hace el parche

Manda la **última sesión en llegar**; las anteriores se retiran en cuanto dan
la siguiente vuelta al bucle. Dos métodos nuevos en `TeleVuer`,
`_toma_el_relevo()` y `_relevado()`, y una comprobación al principio de los
nueve bucles de imagen.

Deja una línea en el log cuando ocurre:

```
[televuer] sesion <id> toma el relevo; la anterior (<id>) deja de mandar imagen.
```

Si esa línea aparece a menudo, hay pestañas de más en el visor.

## Lo que NO arregla

Que el cliente reconecte cada 60 s si el servidor no le manda nada —eso es el
vigilante haciendo su trabajo, y pasa por diseño en `--display-mode
pass-through`, donde no hay imagen que mandar—. Y tampoco cierra el socket TCP
viejo: Vuer lo saca de su diccionario pero no lo cierra, así que quedan
conexiones `ESTAB` colgadas hasta que el cliente las suelta.

**La costumbre correcta sigue siendo tener UNA sola pestaña abierta en el
visor.** El parche es la red por si se olvida.
