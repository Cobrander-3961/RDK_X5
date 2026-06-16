#!/usr/bin/env python3
"""豆包语音对话: 打字 → 豆包 → Edge TTS 播放 (国内可用)"""
import sys, os, subprocess, threading, asyncio, speech_recognition as sr
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from doubao_client import DoubaoClient

API_KEY = os.environ.get("VOLCENGINE_API_KEY", "")
MODEL  = os.environ.get("VOLCENGINE_MODEL", "doubao-1.5-vision-pro-32k")

# ─── TTS: Edge TTS (免费, 国内可用) ───
def _play_tts(text):
    async def _run():
        try:
            await __import__('edge_tts').Communicate(
                text, 'zh-CN-XiaoxiaoNeural').save('/tmp/tts.mp3')
            subprocess.run(['ffplay','-nodisp','-autoexit','-loglevel','quiet','/tmp/tts.mp3'],
                         timeout=15)
        except: pass
    try: asyncio.run(_run())
    except: pass

def speak(text):
    threading.Thread(target=_play_tts, args=(text,), daemon=True).start()

# ─── ASR: arecord + Whisper (离线, 免费, 中文优秀) ───
_whisper_model = None
def _get_whisper():
    global _whisper_model
    if _whisper_model is None:
        import whisper
        print("加载 Whisper tiny 模型...")
        _whisper_model = whisper.load_model("tiny")  # 80MB, CPU可用
    return _whisper_model

def listen():
    wav = "/tmp/voice.wav"
    print("\r🎤 录音中...", end="", flush=True)
    for dev in ["plughw:1,0", "plughw:0,0", "default"]:
        r = subprocess.run(["arecord", "-D", dev, "-c", "1", "-f", "S16_LE",
                           "-d", "4", "-r", "16000", wav], capture_output=True, timeout=6)
        if os.path.exists(wav) and os.path.getsize(wav) > 1000: break
    if not os.path.exists(wav) or os.path.getsize(wav) < 1000:
        print("\r⏰ 没声音"); return None
    try:
        model = _get_whisper()
        result = model.transcribe(wav, language="zh", fp16=False)
        text = result["text"].strip()
        print(f"\r👤 你: {text}"); return text
    except Exception as e:
        print(f"\r💡 Whisper错误: {e}"); return None

def main():
    client = DoubaoClient(api_key=API_KEY, model=MODEL)
    print("=" * 40)
    print("  豆包对话 + 语音播放")
    print("  R=录音 Q=退出  直接打字也可")
    print("=" * 40)

    while True:
        try: cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt): break
        if not cmd: continue
        if cmd.lower() == 'q': speak("再见"); break
        if cmd.lower() == 'r':
            text = listen()
            if not text: continue
        else:
            text = cmd
            print(f"👤 你: {text}")

        print("🤖 思考中...")
        try:
            reply = client.chat(text)
            print(f"🤖: {reply}")
            speak(reply)
        except Exception as e:
            print(f"[API] {e}")

if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt: print("\nBye!")
