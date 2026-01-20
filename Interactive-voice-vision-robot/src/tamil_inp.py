import whisper
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav
import tempfile
import os

print("Loading Whisper model...")
model = whisper.load_model("small")  # medium if GPU / strong CPU
print("Whisper loaded\n")

SAMPLE_RATE = 16000
DURATION = 6  # seconds (Tamil needs a bit more context)

print("Tamil → English Voice Translator (Whisper)")
print("Speak Tamil clearly after 'Recording...'")
print("Ctrl+C to exit\n")

while True:
    try:
        print("🎙 Recording...")
        
        audio = sd.rec(
            int(DURATION * SAMPLE_RATE),
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype="float32"
        )
        sd.wait()

        # Normalize audio
        audio = audio.flatten()
        max_val = np.max(np.abs(audio))
        if max_val > 0:
            audio = audio / max_val

        # Save temp wav
        with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as f:
            wav.write(f.name, SAMPLE_RATE, audio)
            audio_path = f.name

        print("Transcribing & Translating...")

        result = model.transcribe(
            audio_path,
            task="translate",   # auto language detect → English
            beam_size=5,
            temperature=0.0
        )

        os.remove(audio_path)

        text = result["text"].strip()

        if text:
            print("English :", text)
        else:
            print("No meaningful speech detected")

        print("-" * 50)

    except KeyboardInterrupt:
        print("\nExiting...")
        break

    except Exception as e:
        print(f"Error: {e}\n")
