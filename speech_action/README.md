# speech_action
## Overview  
Sample programs for Chapter 3  
Programs for performing speech recognition and speech synthesis using ROS 2 and Whisper.


## Execution
- Speech Recognition (Section 3.1)  
  - Open a terminal and echo the `/speech` topic to check published data:
    ```bash
    ros2 topic echo /speech
    ```
  - Open a new terminal and start the speech recognition server:
    ```bash
    ros2 run speech_action speech_recognition_server
    ```
  - Open another terminal and start the speech recognition client:
    ```bash
    ros2 run speech_action speech_recognition_client
    ```
  - The client will wait in standby. Press Enter to begin.
  - Speak into the microphone.
  - To cancel during input, press `c`.
  
- Speech Synthesis (Section 3.2)  
  - Open a terminal and start the speech synthesis server:
    ```bash
    ros2 run speech_action speech_synthesis_server
    ```
  - Open a new terminal and start the speech synthesis client:
    ```bash
    ros2 run speech_action speech_synthesis_client
    ```
  - Open another terminal and publish a message to the `/speech` topic:
    ```bash
    ros2 topic pub -1 /speech std_msgs/msg/String "{data: 'I will go to the kitchen and grab a bottle.'}"
    ```
  - The message will be spoken aloud through the speaker.

- Speech Echo Using Action Communication (Section 3.3.1)  
  - Open a terminal and start the speech recognition server:
    ```bash
    ros2 run speech_action speech_recognition_server
    ```
  - Open a new terminal and start the speech synthesis server:
    ```bash
    ros2 run speech_action speech_synthesis_server
    ```
  - Open another terminal and start the speech client:
    ```bash
    ros2 run speech_action speech_client
    ```
  - Speak into the microphone. The same phrase will be echoed back through the speaker.
  

## Help
- If you plan to run this sample in a Docker container, please note that it has only been tested with **Ubuntu as the host OS**.  
  For those developing on Windows, you can install Ubuntu on a virtual machine (e.g., VMWare) and run the sample programs there.

- If speech output is not heard during the speech synthesis step, try running `speech_synthesis_server_mpg123.py`.  
  This will generate a synthesized MP3 file which you can manually play to verify the output.

- To change the Whisper model size or recognition language, modify the arguments of `recognize_whisper` inside `recognition.py`.  
  Refer to the following for available models and languages:  
  - Model: [https://github.com/openai/whisper#available-models-and-languages](https://github.com/openai/whisper#available-models-and-languages)  
  - Language: [https://github.com/openai/whisper/blob/main/whisper/tokenizer.py](https://github.com/openai/whisper/blob/main/whisper/tokenizer.py)


## Authors
Yoshinobu Hagiwara 

## Revision History 
- 2026-xx-xx: Initial version #TO DO

## License
Copyright (c) 2026, Yoshinobu Hagiwara, Lucas da Mota Bruno and Jiahao Sim
All rights reserved.
This project is licensed under the Apache-2.0 license found in the LICENSE file in the root directory of this project.

## References 
