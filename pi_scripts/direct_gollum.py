import sys
# Path for your Pi 5 / Python 3.13 packages
sys.path.append('/home/aviv/.local/lib/python3.13/site-packages')

import os
import time
import ctypes
import speech_recognition as sr
from gtts import gTTS
from openai import OpenAI

# --- 1. SILENCE ALSA ERRORS (FIXED VERSION) ---
# Note: we use c_int instead of int here
ERROR_HANDLER_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p)
def py_error_handler(filename, line, function, err, fmt): pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
try:
    asound = ctypes.cdll.LoadLibrary('libasound.so.2')
    asound.snd_lib_error_set_handler(c_error_handler)
except:
    pass

# --- 2. CONFIGURATION ---
# Use your your-openai-api-key-here... API key here
client = OpenAI(api_key='your-openai-api-key-here')
WAKE_WORD = "robot"

def speak(text):
    print(f"AI: {text}")
    try:
        tts = gTTS(text=text, lang='en')
        tts.save("response.mp3")
        os.system("mpg123 response.mp3 > /dev/null 2>&1")
    except Exception as e:
        print(f"TTS Error: {e}")

def get_gpt_response(user_input):
    try:
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": "You are a helpful, concise AI assistant named Robot. You are integrated into a SCUTTLE robotics platform."},
                {"role": "user", "content": user_input}
            ]
        )
        return response.choices[0].message.content
    except Exception as e:
        return f"GPT Error: {e}"

def listen():
    r = sr.Recognizer()
    # device_index=0 for your AIRHUG hardware
    with sr.Microphone(device_index=0) as source:
        print("\n[Listening...]")
        r.adjust_for_ambient_noise(source, duration=0.5)
        try:
            audio = r.listen(source, timeout=5, phrase_time_limit=10)
            text = r.recognize_google(audio).lower()
            return text
        except:
            return ""

if __name__ == "__main__":
    speak("System online. How can I help?")
    
    while True:
        voice_input = listen()
        
        if WAKE_WORD in voice_input:
            query = voice_input.replace(WAKE_WORD, "").strip()
            
            if not query:
                speak("Yes? I am listening.")
            else:
                print(f"Query: {query}")
                answer = get_gpt_response(query)
                speak(answer)
            
        if "shutdown system" in voice_input:
            speak("Powering down. Goodbye.")
            break
            
        time.sleep(0.1)
