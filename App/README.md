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

- **Patient**: Create/select patient (ID, age, gender). Stored in SQLite.
- **Start assessments**: Buttons to “start” Finger Tapping and Pronation–Supination for right hand. In this prototype, clicking adds a mock result (random UPDRS 0–4, duration, today’s date and time).
- **Results**: Table of all test results (test type, UPDRS score, duration, date) and a bar chart comparing left vs right hand scores per test.

Data is stored in `backend/pawd.db` (SQLite).
