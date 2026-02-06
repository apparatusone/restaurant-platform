FROM python:3.13

WORKDIR /app

COPY shared ./shared
COPY services/staff-service/requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY services/staff-service/src ./src

EXPOSE 8001

RUN useradd app
USER app

CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8001"]
