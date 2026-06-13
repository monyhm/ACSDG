FROM python:3.12-slim

WORKDIR /app

# Install deps first for better layer caching
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY app ./app
COPY static ./static

# Data (the SQLite DB) lives here; mount a persistent volume at /data in prod.
ENV BUDGET_DB=/data/budget.db
RUN mkdir -p /data

EXPOSE 8000

# Hosts (Render, etc.) inject $PORT; default to 8000 locally / on Fly.
CMD uvicorn app.main:app --host 0.0.0.0 --port ${PORT:-8000}
