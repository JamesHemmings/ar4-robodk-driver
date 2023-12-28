from voice_2_text import recognize_speech_from_mic
from text_2_speech import text_to_speech

words = recognize_speech_from_mic()

text_to_speech(words)
