#!/usr/bin/env python3.6
import speech_recognition as sr
import rospy
from std_msgs.msg import Int8

# Problems with pyaudio fix: https://stackoverflow.com/questions/20023131/cannot-install-pyaudio-gcc-error


def main():
    #list_microphones()

    global text
    tool_ids = {
        'bowl': 1,
        'tvm': 2,
        'pan': 3,
        'hammer': 4,
        'knife': 5,
        'cup': 6,
        'drill': 7,
        'racket': 8,
        'spatula': 9,
        'bottle': 10
    }

    if not rospy.is_shutdown():
        rospy.init_node('speech_recognition', anonymous=True)
        pub = rospy.Publisher('daniel_change_this_to_tool_id', Int8, queue_size=10)
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            text = None

            while text == None:
                text = speech_to_text()
                print('Text gotten from audio: ' + '\n' + str(text))

            found = look_for_keywords(text, keywords=list(tool_ids))
            print('Found keywords: ' + str(found))

            if len(found) == 1:
                tool = tool_ids[found[0]]
                print('Sending ID: ' + str(tool))

                while pub.get_num_connections() == 0:
                    rate.sleep()

                msg = Int8()
                msg.data = tool
                pub.publish(msg)

            else:
                print('Found more or less than 1 tool to pick')


def speech_to_text():
    r = sr.Recognizer()
    r.energy_threshold = 400

    with sr.Microphone() as source:
        print("Adjusting to background noise...")
        r.adjust_for_ambient_noise(source=source, duration=1)
        print("Say something...")
        audio = r.listen(source=source, phrase_time_limit=3)
        print("Got something...")

    try:
        return r.recognize_google(audio)
    except sr.UnknownValueError:
        print('Could not get any words from audio')
        return None


def look_for_keywords(text, keywords):
    found_keywords = []
    for word in keywords:
        if word in text:
            found_keywords.append(word)

    return found_keywords


def list_microphones():
    for index, name in enumerate(sr.Microphone.list_microphone_names()):
        print("Microphone with name \"{1}\" found for `Microphone(device_index={0})`".format(index, name))


if __name__ == '__main__':
    main()

