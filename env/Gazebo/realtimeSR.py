import random
import time
import speech_recognition as sr
import tensorflow as tf


def recognize_tensorflow(audio_data, tensor_graph='conv_actions_frozen.pb',
                         tensor_label='conv_actions_labels.txt'):
    """
    Performs speech recognition on ``audio_data`` (an ``AudioData`` instance).
    Path to Tensor loaded from ``tensor_graph``. You can download a model here: http://download.tensorflow.org/models/speech_commands_v0.01.zip
    Path to Tensor Labels file loaded from ``tensor_label``.
    """
    assert isinstance(tensor_graph, str), "``tensor_graph`` must be a string"
    assert isinstance(tensor_label, str), "``tensor_label`` must be a string"

    lasttfgraph = ''
    tflabels = None

    if not (tensor_graph == lasttfgraph):
        lasttfgraph = tensor_graph

        # load graph
        with tf.gfile.FastGFile(tensor_graph, 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
            tf.import_graph_def(graph_def, name='')
        # load labels
        tflabels = [line.rstrip() for line in tf.gfile.GFile(tensor_label)]

    wav_data = audio_data.get_wav_data(
        convert_rate=16000, convert_width=2
    )

    with tf.Session() as sess:
        input_layer_name = 'wav_data:0'
        output_layer_name = 'labels_softmax:0'
        softmax_tensor = sess.graph.get_tensor_by_name(output_layer_name)
        predictions, = sess.run(softmax_tensor, {input_layer_name: wav_data})

        # Sort labels in order of confidence
        top_k = predictions.argsort()[-1:][::-1]
        for node_id in top_k:
            human_string = tflabels[node_id]
            return human_string

if __name__ == "__main__":

    names = ["Rachel", "Joey", "Phoebe", "Catherine", "Monica", "Chandler", "Ross"]
    num_people = 7
    prompt_limit = 5
    word = "Ross"

    # create recognizer and mic instances
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    # format the instructions string
    instructions = (
        "Choose a person you want me to point at:\n"
        "{words}\n"
        "You have {n} people to choose from.\n"
    ).format(words=', '.join(names), n=num_people)

    # show instructions and wait 3 seconds before starting the game
    print(instructions)
    time.sleep(3)
    word = "Ross"

    for i in range(prompt_limit):
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

        wav_data = audio.get_wav_data(
            convert_rate=16000, convert_width=2
        )

        print(wav_data)
        # set up the response object
        response = {
            "success": True,
            "error": None,
            "transcription": None
        }

        try:
            response["transcription"] = recognize_tensorflow(audio)
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


        # show the user the transcription
    print("You said: {}".format(guess["transcription"]))

    if guess["transcription"] in names:
        print(guess["transcription"])
    else:
        print("Sorry,'{}' is not in this picture.".format(guess["transcription"]))
