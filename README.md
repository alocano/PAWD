<h1 align="center">
Parkinson's Assistive Wearable Device (PAWD)
</h1>
<p align="center">
  <img src="App/frontend/logo.png" alt="PAWD Logo" width=300>
</p>

## About
The Parkinson's Assistive Wearable Device (PAWD) is a glove designed to observe motor function
and the possibility of dyskensia of a person with Parkinson's Disease. The current version of
PAWD is designed to be implemented in clinical settings during a routine visit as a tool during
a symptom assessment with scoring based on the Unified Parkinson's Disease Rating Scale (UPDRS),
specifically the Finger Tapping (Section 3.4 of the UPDRS) and Pronation-Supination (Section 3.6)
tests.

PAWD captures motion data and rapidly returns that collected data in graph form to allow both
patient and clinician understanding of movement to a more precise scale, assisting in observing
movement slowing, abruptions, or freezing.

This project was developed for a senior capstone project over the course of two semesters with collaboration from CSUS faculty from
the Kinesiology and Electrical & Electronics Engineering departments who provided guidance on technical
decisions, clinical relevance, and IRB compliance steps.

### Why PAWD?
Patients, especially older patients in more rural or remote areas, struggle to access the care they need, and for patients with Parkinson's Disease, this can impact some aspects of everyday life. PAWD was made to address this gap, originally designed as an affordable at-home device that will allow doctors to view symptom severity trends more often over time and assist patients with communication and reminders without worrying about when their next visit will be.

### Limitations
This project faced a lot of issues due to time and money as a mostly self-funded university project:
- **Limited Validation**: Testing was conducted in an IRB approved environment, providing early real-world signal but is not sufficient for validation against existing PD assessment methods (e.g. UPDRS, MOCA, other wearables)
- **Hardware limitations**: A common problem that arose was effective hardware for our tests, specifically for the finger taps test. As we shifted to a force sensitive resistor, we were able to find something sensitive enough for the test, but not large enough to account for things such as tap location, varying hand size, etc.
- **Wearability**: The current design only covers the right hand, not accounting for a realistic clinical setting in which a patient will need to test on both hands.
### Future Implementations
Potential growth for continued development:
- **Mobile Compatibility**: Swapping from solely web to mobile will be more accessible, especially for patients who may struggle using computers, especially mouses.
- **BLE  migration**: Adjusting to BLE will allow for more mobile-friendly use
- **Refined device design**: Enclosure redesign for improved comfort during extended wear.

## Tech Stack
#### Hardware
| COMPONENT                       	| PART                                         	| NOTES                                                                                                                                                                  	|
|---------------------------------	|----------------------------------------------	|------------------------------------------------------------------------------------------------------------------------------------------------------------------------	|
| Microcontrollers                	| ESP32 WROOM                                  	| The ESP32 WROOM was used in a dongle ("PAWD Caster") connected to a PC to store sensor data in a plaintext file that would then be displayed on the PAWD portal                                                                                        	|
|                                 	| Xiao ESP32 S3                                	| The Xiao was the microcontroller on the wearable, used for transmitting sensor data, communicating with the"PAWD caster"                                                                                                                                    	|
| Force Sensor (FSR)              	| Force Sensitive Resistor                     	| The FSR was for the Finger Tapping test, capturing ADC values on tap and recording time between each                                                                   	|
| Intertial Measurment Unit (IMU) 	| Adafruit LSM6DS3TR-C                         	| The IMU was used for the Pronation-Supination test, capturing rotation times/degrees per second to detect one full hand rotation and time between each                 	|
| TFT Screen w/ Encoder           	| 2.4 Inch TFT Screen With EC11 Rotary Encoder 	| This screen was used on the PAWD Caster to display the test selection screen and real-time count of each tap/rotation and allow users to choose between the two tests. 	|
| PAWD Caster                     	| 3D Printed Enclosure                         	|                                                                                                                                                                        	|

#### Software
Backend: Python (Flask)
Frontend: HTML, CSS, JavaScript (ChartJS for visualization)

#### System Flow
Test Selection -> Initializing -> Finger Tapping or Pronation-Supination Test  -> Transmit data back to PAWD Caster ->
Select file w collected data

### Repository Structure
#### Note: Repo has been reorganized due to duplicate or outdated folders/files. Unused/Older code is in Archive/ for reference and some Arduino files + folders must be named/renamed accordingly.
```
├──App
│   ├── Frontend/
│   ├── Backend/
│   ├── SendReceive-Files/
│   ├── README.md
├── archive/ # includes first PAWD prototype from Fall 2025 semester & original code from beginning of Spring 2026 smester
│
├── docs/
│   ├── ShowcaseSlides.pptx		 # Spring semester slides
│   └── meeting-notes/           # first semester meeting notes 
│
├── Glove/
│   ├── ESPNOWTFT_PAWD_mini
│   ├── ESPNOWTFT_PAWD_wroom #ESPNOW code for board communication
│   ├── Glove/
│   	├── GLOVE_PAWD	#Final dongle/PAWD Caster code
│		└── Mac_Address_Finder # MAC Address finder to put into the dongle code
└── README.md
```
## How to Run PAWD
### Requirements
- Arduino IDE
- Python

### Device
The hardware code can be run in Arduino IDE and VSCode, make sure you use the appropriate files (.ino in Arduino IDE and C++ in VSCode) and have the necessary extensions (VSCode).
### receive.py
Be sure you are in the correct directory and use the following command to run the **receive.py** program
```bash
python receive_v3.py --port COM11 --baud 115200 --duration 30 --patient_id P001 --test_type pronation_supination
```
### Web App

Open a new terminal and make sure you are in the correct directory to run the following:
1. Install dependencies
```bash
cd App/backend
```

```bash
pip install -r requirements.txt
```

If that doesn't work, type in each requirement:

```bash
pip install requests pyserial flask
```
2. Start the server
```bash 
python app.py
```

3. Open in a browser: **http://127.0.0.1:5000**

4. Begin running the tests, and when complete (either from completing the full test, running out of time, or manually ending in the terminal), the data will be stored in a .txt file in the backend and can be displayed on the website.
Tests can be ended manually using CTRL+C where receive_v3.py is being run and the file will still be saved.
