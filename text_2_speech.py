from gtts import gTTS
from playsound import playsound
import os


def text_to_speech(text, lang='en'):
    tts = gTTS(text=text, lang=lang, tld='com.au')  # Convert text to speech
    filename = 'temp_audio.mp3'  # Temporary file to store audio
    tts.save(filename)  # Save the converted audio to a file
    playsound(filename)  # Play the audio file
    os.remove(filename)  # Remove the temporary file


# Example usage
if __name__ == "__main__":
    text = "fuck you stupid fucker"
    text_to_speech(text)
