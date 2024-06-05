from selenium import webdriver
from selenium.webdriver.common.by import By
import time

try:
    # Set up the webdriver to connect to the remote Selenium server
    options = webdriver.ChromeOptions()
    options.add_argument('--no-sandbox')
    options.add_argument('--disable-dev-shm-usage')
    
    # Disable headless mode to show the Chrome UI
    # options.add_argument('--headless=false')

    # Remote WebDriver URL (provided by the selenium/standalone-chrome service)
    driver = webdriver.Remote(
        command_executor='http://localhost:4444/wd/hub',
        options=options
    )

    # Open the browser and go to the URL
    driver.get('https://google.com')

    # Wait for the page to load
    time.sleep(40)


    # Capture a screenshot
    driver.get_screenshot_as_file('screenshot.png')

finally:
    # Close the browser to ensure the session is properly terminated
    if driver:
        driver.quit()
