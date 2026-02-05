FROM node:22-alpine

WORKDIR /app

COPY apps/pos-system/package*.json ./
RUN npm ci

COPY apps/pos-system/ ./

RUN npm run build

EXPOSE 3000

RUN adduser -D app
USER app

CMD ["node", "build"]
