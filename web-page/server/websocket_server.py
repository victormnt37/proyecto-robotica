import asyncio
import websockets
import subprocess
import json

async def handler(websocket):
    print("Cliente Web conectado")
    try:
        async for message in websocket:
            print(f"Mensaje recibido: {message}")
            try:
                data = json.loads(message)
                if data["command"] == "waypoint_routine":
                    print("Ejecutando rutina de waypoints...")
                    cmd = "cd ~/proyecto-robotica && source install/setup.bash && ros2 run kyron_nav kyron_nav_wf"
                    subprocess.Popen(["bash", "-c", cmd])
                    await websocket.send("Rutina de waypoints iniciada")

                elif data["command"] == "go_to_goal":
                    x = str(data["x"])
                    y = str(data["y"])
                    w = str(data["w"])
                    cmd = f"cd ~/proyecto-robotica && source install/setup.bash && ros2 run kyron_nav kyron_goal_pub ({x}, {y}, {w})"
                    subprocess.Popen(["bash", "-c", cmd])
                    await websocket.send(f"Goal enviado a ({x}, {y}, {w})")

                else:
                    await websocket.send("Comando desconocido")

            except (json.JSONDecodeError, KeyError) as e:
                print(f"Error en el mensaje: {e}")
                await websocket.send("Error: formato inválido")
    except websockets.exceptions.ConnectionClosed:
        print("Cliente Web desconectado")


async def main():
    print("Servidor WebSocket escuchando en puerto 8765...")
    async with websockets.serve(handler, "0.0.0.0", 8765):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
