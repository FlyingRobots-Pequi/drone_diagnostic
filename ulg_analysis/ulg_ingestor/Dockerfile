FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY ulg_ingestor.py .

CMD ["python", "ulg_ingestor.py"]
