import speech_recognition as sr


def recognize_speech_from_mic():
    # Initialize recognizer
    recognizer = sr.Recognizer()

    # Use the default microphone as the audio source
    with sr.Microphone() as source:
        print("Listening...")
        try:
            # Adjust for ambient noise and record audio
            recognizer.adjust_for_ambient_noise(source)
            audio = recognizer.listen(source)

            # Recognize speech using Google Web Speech API
            text = recognizer.recognize_google(audio)
            print("You said: " + text)
        except sr.UnknownValueError:
            # Error: recognizer could not understand audio
            print("Could not understand audio")
        except sr.RequestError as e:
            # Error: could not request results from Google Web Speech API
            print(f"Could not request results; {e}")
    if text:
        return text


if __name__ == "__main__":
    recognize_speech_from_mic()
