#!/bin/bash
set -e

echo "Starting backend server on port 8080..."
cd /app/VisualCircuit/backend
python3 manage.py migrate || true
python3 manage.py runserver 0.0.0.0:8080 &

echo "Starting frontend server on port 4000..."
cd /app/VisualCircuit/frontend
npm start
