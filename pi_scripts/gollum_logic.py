import sys
sys.path.append('/home/aviv/.local/lib/python3.13/site-packages')

import asyncio
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.proto.common import ResourceName # This fixes the 'str' error

async def connect():
    opts = RobotClient.Options(
        refresh_interval=0,
        dial_options=DialOptions(
            credentials=Credentials(
                type="api-key",
                payload="ahkuy568gka91qfx5a92fynxaestsfkv"
            ),
            auth_entity="889f682c-c5da-4b4b-99cf-d8c120720b8b",
            insecure=True
        )
    )
    return await RobotClient.at_address("scuttle-main.clq8184f3f.viam.cloud", opts)

async def main():
    print("--- Connecting to SCUTTLE v3 ---")
    try:
        robot = await connect()
        print("--- Connected Successfully! ---")

        # This is the "old reliable" way to get services
        stt = robot.get_component("speech-1")
        tts = robot.get_component("speech-2")
        while True:
            # LOOK HERE: This is where the terminal will wait for you
            input("\n[Press ENTER here and speak into the Airhug]")
            
            try:
                stt_resp = await stt.do_command({"command": "to_text"})
                text = stt_resp.get("text", "")
                
                if text:
                    print(f"Heard: {text}")
                    if "precious" in text.lower():
                        await tts.do_command({"command": "say", "text": "My preciousssss!"})
                else:
                    print("Gollum heard nothing.")
                    
            except Exception as e:
                print(f"Module error: {e}")

    except Exception as e:
        print(f"Connection error: {e}")
    finally:
        if 'robot' in locals():
            await robot.close()

if __name__ == "__main__":
    asyncio.run(main())
