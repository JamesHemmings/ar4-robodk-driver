import re
from voice_2_text import recognize_speech_from_mic
from text_2_speech import text_to_speech
import spacy

# Load the English language model
nlp = spacy.load("en_core_web_lg")

# List of available commands
commands = [
    "home robot",
    "move joint",
    "set work offset",
    "run program",
    "stop program",
    "jog robot",
]

# Function to find the best matching command
def find_best_match(input_text):
    input_doc = nlp(input_text)
    best_match = None
    best_similarity = 0

    for command in commands:
        command_doc = nlp(command)
        similarity = input_doc.similarity(command_doc)
        if similarity > best_similarity:
            best_similarity = similarity
            best_match = command

    return best_match

# Function to extract coordinates from the input
def extract_coordinates(input_text):
    coordinates = re.findall(r'[XYZxyz](\d+)', input_text)
    return coordinates if len(coordinates) == 3 else None

# Example usage
print("Please speak your command:")
user_input = recognize_speech_from_mic()

coordinates = extract_coordinates(user_input)

best_match = find_best_match(user_input)

# Check if the best match is "jog robot" and if coordinates are present in the input
if best_match == "jog robot" and coordinates:
    best_match = f"jog robot to (X{coordinates[0]}, Y{coordinates[1]}, Z{coordinates[2]})"

print("Best matching command:", best_match)
text_to_speech(best_match)