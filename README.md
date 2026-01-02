# Chapter 3: Speech Recognition and Synthesis (Revised 2nd Edition)
## Overview
This repository provides sample programs and supplementary information for Chapter 3.

## Installation
- Install libraries required for handling audio using the following commands.
```
sudo apt install portaudio19-dev
sudo apt install pulseaudio
```
- To use them as Python modules, run the following command.
```
pip3 install pyaudio
```
- Install a speech recognition library using the following command.
```
pip3 install SpeechRecognition
```
- Install libraries required to use Whisper as a speech recognizer using the following command.
```
pip3 install SpeechRecognition[whisper-local] soundfile
```
- Install libraries used for speech synthesis.
```
pip3 install gTTS
sudo apt install mpg123
pip3 install mpg123
```
- Clone the sample programs from GitHub using the following command.
```
cd ~/airobot_ws/src
git clone https://github.com/AI-Robot-Book-Humble/chapter3
```
- Build the package using the following commands.
```
cd ~/airobot_ws
colcon build
source install/setup.bash
```

## Directory Structure
- **[speech_action](speech_action):** Sample programs for speech recognition and speech synthesis using action communication
- **[speech_service](speech_service):** Sample programs for speech recognition and speech synthesis using service communication
- **[speech_topic](speech_topic):** Sample programs for speech recognition and speech synthesis using topic communication

## Additional Notes
- When running the sample programs from Chapter 3 in a Docker container, **we have confirmed operation only when Ubuntu is used as the host OS**. If you are developing on Windows, you can install Ubuntu in a virtual machine such as VMware and run the sample programs there.
- Before running the programs, make sure that your Ubuntu environment can receive audio input from the microphone and output sound through the speakers.
- If you are using Ubuntu in a virtual machine, latency may occur and audio may not be output. In that case, try using longer utterances as a workaround.
- If possible, please use a headset. During echo/back (repeat-after-me) execution, the speaker output may be picked up by the microphone and cause repeated feedback.
