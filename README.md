# Sound-Locator
Orients a turret in the direction of a loud sound. Scans and follows nearby objects.

### Summary

This project explores the feasibility of using two analog microphones and an ESP32 to estimate the direction of a sound source by computing the time difference of arrival (TDOA) between the channels. The goal was to create a low-cost, real-time audio direction tracker using built-in ADCs and DMA sampling. The system samples both microphones, measures the signal envelopes, and triggers a comparator with tunable reference to measure TDOA and calculate the angle of origination of the sound.


The prototype partially works: it correctly detects Δt for sharp, isolated sounds but is sensitive to echo, gain mismatches, and low signal-to-noise conditions. Further tuning and hardware upgrades are needed for robust directional detection.

### Design

The system consists of:

Two analog microphones with MAX9814 preamps
An ESP32 microcontroller running at 240 MHz
A tunable envelope detector 
A stepper motor and driver for orientation

The microphones are placed 17 cm apart, and sounds arriving from an angle create a small but measurable delay between 1us and 516us between the two signals. Dual channel sampling with cross correlation was explored however not implemented due to limitations of the onboard ADC of the ESP32.


<img width="1486" height="348" alt="MicInputsAnd ComparatorOutputsWithoutEnvelopets" src="https://github.com/user-attachments/assets/2e50f9fc-7c3f-47f1-9a0f-f977b92d57f6" />

Figure 1: Conversion of two audio signals into threshold detectors

<img width="633" height="290" alt="Adding a 100uF cap to the rectifier output" src="https://github.com/user-attachments/assets/b2e89601-d467-4259-a52e-c23ebbd66cb1" />


Figure 2: Implementing a low pass filter and smoothing cap to the comparator output

<img width="736" height="159" alt="Left mic" src="https://github.com/user-attachments/assets/884b6bc7-bf6c-4e45-9922-bbdcde628ed0" />

Figure 3: A response to a sound aimed at one microphone

<img width="632" height="168" alt="Middle" src="https://github.com/user-attachments/assets/4110fd54-f572-4f6a-9ff8-b5043678de8c" />

Figure 4: A repsonse to a sound coming from between microphones

### Circuit

The circuit diagram and pcb used in the prototype are below.

<img width="3507" height="2480" alt="Untitled" src="https://github.com/user-attachments/assets/94d388ef-00fb-4ba1-a390-216dd17175db" />

Figure 4: Circuit diagram

### Results

The prototype produces consistant readings in quiet environments and has an adjustable noise floor. The agc of the MAX9814 was detrimental in matching proper rise times for the signal envelopes as the amplitude of each signal could vary widely with loud noises. A prototype was constructed however is not accurate enough and i will need to go back to the drawing board. The prototype can respond reliably to knocking from 20 feet away in a quiet room. The analog mics will need to be replaced with digital MEMS without agc and an alternative high sampling 2 channel ADC would make the results much more accurate.


![image0](https://github.com/user-attachments/assets/db208b19-997b-40a0-8862-4c0d5f2a7929)

Figure 5: Proof of concept prototype
