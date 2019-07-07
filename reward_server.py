import asyncio
import json
import traceback
import reward_function as rf

port = 11111
waypoints = []

async def handle_stream(reader, writer):
    global waypoints
    try:
        print("Connected.")
        while True:
            readbytes = await reader.readline()
            if not readbytes:
                break

            request = json.loads(readbytes.decode('utf-8'))
            op = request['op']
            if op == "InitializingData":
                print("Got InitializingData")
                waypoints = []
                for wp in request['waypoints']:
                    waypoints.append((wp['X'], wp['Y']))
            elif op == "RewardRequest":
                response = get_reward(request)
                await send(writer, response)
            else:
                print("Got Unknown {}".format(request))
    except Exception:
        traceback.print_exc()

    print("Closing socket.")
    writer.close()

async def send(writer, response):
    writer.write((json.dumps(response) + "\n").encode('utf-8'))
    await writer.drain()

def get_reward(request):
    global waypoints
    request['waypoints'] = waypoints
    response = {
        "op": "RewardResponse",
        "reward": rf.reward_function(request),
        "ext_rewards": {}
    }

    for fname in dir(rf):
        if not fname.endswith("_reward"):
            continue

        func = getattr(rf, fname)
        response["ext_rewards"][fname] = func(request)

    return response

        

loop = asyncio.get_event_loop()
coro = asyncio.start_server(handle_stream, '127.0.0.1', port, loop = loop)
server = loop.run_until_complete(coro)

# Serve requests until Ctrl+c is pressed
print('Serving on {}'.format(server.sockets[0].getsockname()))
try:
    loop.run_forever()
except KeyboardInterrupt:
    pass

# Close the server
server.close()
loop.run_until_complete(server.wait_closed())
loop.close()