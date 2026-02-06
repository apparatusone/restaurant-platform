FROM python:3.13

WORKDIR /app

COPY shared ./shared
COPY services/payment-service/requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY services/payment-service/src ./src

EXPOSE 8004

RUN useradd app
USER app

CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8004"]
