import random
import time
import speech_recognition as sr

class speechRec(object):
    def __init__(self):
        self.names = ["Rachel", "Joey", "Phoebe", "Catherine", "Monica","Chandler", "Ross"]
        self.num_people = 7
        self.prompt_limit= 5
        self.word = "Ross"

    def init(self):
        # create recognizer and mic instances
        recognizer = sr.Recognizer()
        microphone = sr.Microphone()

        # format the instructions string
        instructions = (
            "Choose a person you want me to point at:\n"
            "{words}\n"
            "You have {n} people to choose from.\n"
        ).format(words=', '.join(self.names), n=self.num_people)

        # show instructions and wait 3 seconds before starting the game
        print(instructions)
        time.sleep(3)
        self.word = "Ross"

        return recognizer, microphone

    def startGame(self,recognizer, microphone):
        for i in range(self.prompt_limit):
            print('Guess {}. Speak!'.format(i + 1))
            if not isinstance(recognizer, sr.Recognizer):
                raise TypeError("`recognizer` must be `Recognizer` instance")

            if not isinstance(microphone, sr.Microphone):
                raise TypeError("`microphone` must be `Microphone` instance")

            # adjust the recognizer sensitivity to ambient noise and record audio
            # from the microphone
            with microphone as source:
                recognizer.adjust_for_ambient_noise(source)
                audio = recognizer.listen(source)

            # set up the response object
            response = {
                "success": True,
                "error": None,
                "transcription": None
            }

            # try recognizing the speech in the recording
            # if a RequestError or UnknownValueError exception is caught,
            #     update the response object accordingly
            try:
                response["transcription"] = recognizer.recognize_google(audio)
            except sr.RequestError:
                # API was unreachable or unresponsive
                response["success"] = False
                response["error"] = "API unavailable"
            except sr.UnknownValueError:
                # speech was unintelligible
                response["error"] = "Unable to recognize speech"
            guess = response
            if guess["transcription"]:
                break
            if not guess["success"]:
                break
            print("I didn't catch that. What did you say?\n")
                # if there was an error, stop the game
        if guess["error"]:
            print("ERROR: {}".format(guess["error"]))
            return False

            # show the user the transcription
        print("You said: {}".format(guess["transcription"]))

        if guess["transcription"] in self.names:
            return guess["transcription"]
        else:
            print("Sorry,'{}' is not in this picture.".format(guess["transcription"]))
            return False

        return False


