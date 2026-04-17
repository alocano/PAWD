# PAWD App (Prototype)

Basic prototype with no hardware communication.

## How to Run the app

1. From the project root, install dependencies:

   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. Start the server:

   ```bash
   python app.py
   ```

3. Open in a browser: **http://127.0.0.1:5000**

## What’s in the prototype

- **Raw sensor data**: Plots the gyroscope waveform with rotation markers and a separate tap timeline.
- **Summary**: Shows tap count, rotation count, and total test duration below the charts.

The app reads from `backend/mydata.txt`; it does not store patient records in this prototype.
