import asyncio
import websockets
import cv2
import numpy as np

async def handler(websocket, path):
    async for message in websocket:
        # message is expected to be JPEG bytes
        nparr = np.frombuffer(message, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        cv2.imshow("Robot Stream", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

async def main():
    async with websockets.serve(handler, "0.0.0.0", 8765):
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())