FROM python:3.7-slim

RUN apt-get update && apt-get install
RUN apt-get install -y build-essential
RUN apt-get -y install python3-dev
RUN apt-get install -y libpq-dev
RUN mkdir /code
COPY . /code/
WORKDIR /code
RUN pip install --upgrade pip
RUN pip install wheel
RUN pip install -r requirements.txt
WORKDIR /code
EXPOSE 8000
CMD ["sh", "-c", "python manage.py collectstatic --noinput && python manage.py migrate && gunicorn backend.wsgi --bind 0.0.0.0:8000"]