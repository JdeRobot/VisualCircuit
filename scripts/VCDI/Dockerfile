FROM python:3.11-slim

RUN apt-get update && apt-get install -y \
    git \
    curl \
    build-essential \
    gnupg \
    && rm -rf /var/lib/apt/lists/*


RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - && \
    apt-get update && apt-get install -y nodejs


WORKDIR /app


ARG BRANCH=master

RUN git clone --branch ${BRANCH} --single-branch https://github.com/JdeRobot/VisualCircuit.git



WORKDIR /app/VisualCircuit/frontend
RUN npm install --legacy-peer-deps


WORKDIR /app/VisualCircuit/backend
RUN pip install --upgrade pip && pip install -r requirements.txt

RUN echo "DJANGO_DEBUG=true" >> /app/VisualCircuit/backend/.env && \
    echo "VISUAL_CIRCUIT_FRONTEND_HOST='http://localhost:4000'" >> /app/VisualCircuit/backend/.env && \
    echo "DJANGO_ALLOWED_HOSTS=localhost,127.0.0.1" >> /app/VisualCircuit/backend/.env && \
    echo "DJANGO_SECRET_KEY=xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" >> /app/VisualCircuit/backend/.env

# Collect static files
RUN python manage.py collectstatic 

WORKDIR /app/VisualCircuit
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 4000 8080
ENTRYPOINT ["/entrypoint.sh"]

