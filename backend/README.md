# VisualCircuit Backend

Django based back-end application for VisualCircuit3. 

## Setup for development

1. Clone the repository https://github.com/JdeRobot/VisualCircuit.git
2. Change directory to `backend`
3. Create a Python3 virtual environment using venv. 
For eg. `python -m venv .venv` 
4. After activating the virtual environment, install the dependencies by running
`pip install -r requirements.txt`
5. Add `.env` file to the `backend` folder. And add the variables as defined in [.env.template](./.env.template)
6. Create the static files to serve during execution by `python manage.py collectstatic`
7. Start the server by running `python manage.py runserver 8080`