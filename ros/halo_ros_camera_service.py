"""Fresh four-camera HTTP bridge. V2 cameras only; never starts robot control."""
import argparse
import asyncio
from aiohttp import web
from camera_preview import PreviewClient


async def application(console):
    client=PreviewClient(console)
    await client.open()
    for _ in range(100):
        try:
            await client.snapshot()
            break
        except RuntimeError as exc:
            if str(exc)!='Waiting for all four house cameras': raise
            await asyncio.sleep(.1)
    else: raise RuntimeError('Camera bridge did not receive all four tracks')
    app=web.Application()
    async def snapshot(_):
        try: return web.json_response(await client.snapshot())
        except Exception as exc: return web.json_response({'error':str(exc)},status=503)
    async def health(_):
        try:
            value=await client.snapshot(False)
            return web.json_response(dict(value,ready=True,robot_control=False))
        except Exception as exc: return web.json_response({'ready':False,'error':str(exc)},status=503)
    async def close(_): await client.close()
    app.router.add_get('/snapshot',snapshot); app.router.add_get('/health',health)
    app.on_cleanup.append(close)
    return app


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--console-url',default='http://127.0.0.1:6657')
    parser.add_argument('--port',type=int,default=5000)
    args=parser.parse_args()
    web.run_app(application(args.console_url),host='127.0.0.1',port=args.port)
