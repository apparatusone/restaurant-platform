FROM python:3.13

WORKDIR /app

COPY shared ./shared
COPY services/api-gateway/requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY services/api-gateway/src ./src

EXPOSE 8000

RUN useradd app
USER app

CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
