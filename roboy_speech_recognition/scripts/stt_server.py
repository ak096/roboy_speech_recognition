#!/usr/bin/env python

import rospy
from multiprocessing import Process, Queue
import webrtcvad
import collections
import os
import sys
import signal
import pyaudio
import wave

import traceback
import pdb 

abs_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(abs_path, "..", "..", "common"))

from bing_voice import *
from roboy_communication_cognition.srv import RecognizeSpeech
from roboy_communication_control.msg import ControlLeds
from std_msgs.msg import Empty

import speaker-recognition as sr

WAV_PATH = "/home/akshay/hackroboy/src/roBauss/speaker-recognition/real_test/speaker.wav"
MODEL_PATH = "/home/akshay/hackroboy/src/roBauss/speaker-recognition/src/model.out"


BING_KEY = ''

def stt_with_vad(bing):
    print('stt_with_vad(bing)')
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    CHUNK_DURATION_MS = 30  # supports 10, 20 and 30 (ms)
    PADDING_DURATION_MS = 1000
    CHUNK_SIZE = int(RATE * CHUNK_DURATION_MS / 1000)
    CHUNK_BYTES = CHUNK_SIZE * 2
    NUM_PADDING_CHUNKS = int(PADDING_DURATION_MS / CHUNK_DURATION_MS)
    NUM_WINDOW_CHUNKS = int(240 / CHUNK_DURATION_MS)

    vad = webrtcvad.Vad(2)
    # bing = BingVoice(BING_KEY)

    pa = pyaudio.PyAudio()
    stream = pa.open(format=FORMAT,
                     channels=CHANNELS,
                     rate=RATE,
                     input=True,
                     start=False,
                     #input_device_index=0,
                     frames_per_buffer=CHUNK_SIZE)


    got_a_sentence = False
    leave = False


    def handle_int(sig, chunk):
        print('handle_int(sig, chunk)')
        global leave, got_a_sentence

        leave = True
        got_a_sentence = True
        
    signal.signal(signal.SIGINT, handle_int)

    while not leave:
        ring_buffer = collections.deque(maxlen=NUM_PADDING_CHUNKS)
        triggered = False
        voiced_frames = []
        ring_buffer_flags = [0] * NUM_WINDOW_CHUNKS
        ring_buffer_index = 0
        buffer_in = ''
        
        print("* recording")
        stream.start_stream()
        while not got_a_sentence: #and not leave:
            chunk = stream.read(CHUNK_SIZE)
            active = vad.is_speech(chunk, RATE)
            sys.stdout.write('1' if active else '0')
            ring_buffer_flags[ring_buffer_index] = 1 if active else 0
            ring_buffer_index += 1
            ring_buffer_index %= NUM_WINDOW_CHUNKS
            if not triggered:
                ring_buffer.append(chunk)
                num_voiced = sum(ring_buffer_flags)
                if num_voiced > 0.5 * NUM_WINDOW_CHUNKS:
                    sys.stdout.write('+')
                    triggered = True
                    voiced_frames.extend(ring_buffer)
                    ring_buffer.clear()
            else:
                voiced_frames.append(chunk)
                ring_buffer.append(chunk)
                num_unvoiced = NUM_WINDOW_CHUNKS - sum(ring_buffer_flags)
                if num_unvoiced > 0.9 * NUM_WINDOW_CHUNKS:
                    sys.stdout.write('-')
                    triggered = False
                    got_a_sentence = True

            sys.stdout.flush()

        sys.stdout.write('\n')
        data = b''.join(voiced_frames)


        wav_file = wave.open("/home/akshay/hackroboy/src/roBauss/speaker-recognition/real_test/speaker", 'wb')
        wav_file.setnchannels(CHANNELS)
        wav_file.setsampwidth(pyaudio.PyAudio().get_sample_size(FORMAT))
        wav_file.setframerate(RATE)
        wav_file.writeframes(data)
        wav_file.close()

        stream.stop_stream()
        print("* done recording")

        # recognize speech using Microsoft Bing Voice Recognition
        '''
        try:
            # pdb.set_trace()
            text = bing.recognize(data, language='en-US')
            # pdb.set_trace()
            print('Bing:' + text.encode('utf-8'))
            stream.close()
            return text
        except UnknownValueError:
            traceback.print_exc()
            print("Microsoft Bing Voice Recognition could not understand audio")
        except RequestError as e:
            print("Could not request results from Microsoft Bing Voice Recognition service; {0}".format(e))
        '''

        text = "Hello," + sr.task_predict(WAV_PATH, MODEL_PATH)
        got_a_sentence = False



    stream.close()
    return text

def stt_subprocess(q):
    print('stt_subprocess(q)')
    q.put(stt_with_vad(bing))

def handle_stt(req):
    print('handle_stt(req)')
    msg = ControlLeds()
    msg.mode=2
    msg.duration=0
    ledmode_pub.publish(msg)
    queue = Queue()
    p = Process(target = stt_subprocess, args = (queue,))
    p.start()
    p.join()
    msg = Empty()
    ledfreeze_pub.publish(msg)
    return queue.get()
        #return stt_with_vad(bing)
def stt_server():
    print('stt_server()')
    #rospy.init_node('roboy_speech_recognition')
    s = rospy.Service('/roboy/cognition/speech/recognition', RecognizeSpeech, handle_stt)
    global ledmode_pub
    ledmode_pub = rospy.Publisher("/roboy/control/matrix/leds/mode", ControlLeds, queue_size=3)
    global ledoff_pub
    ledoff_pub = rospy.Publisher('/roboy/control/matrix/leds/off', Empty, queue_size=10)
    global ledfreeze_pub
    ledfreeze_pub = rospy.Publisher("/roboy/control/matrix/leds/freeze", Empty, queue_size=1)
    rospy.init_node('roboy_speech_recognition')
    global bing
    #bing = BingVoice(BING_KEY)
    bing = 1
    #print(stt_with_vad(0))
    print "Ready to recognise speech."
    rospy.spin()

if __name__ == '__main__':
    print('main')
    stt_server()
