from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
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
    driver.get('http://IP:4000')

    # time.sleep(120)


    # Wait for the "File" button to be clickable and click it
    basic_button = WebDriverWait(driver, 20).until(
        EC.element_to_be_clickable((By.XPATH, "//button[contains(@class, 'menu-button') and .//span[text()='File']]"))
    )
    basic_button.click()


    # Wait for the dropdown menu to be visible
    dropdown_menu = WebDriverWait(driver, 10).until(
        EC.visibility_of_element_located((By.XPATH, "//ul[@role='menu' and @aria-label='File']"))
    )

    # Wait for the "Open" menu item to be clickable and click it
    code_menu_item = WebDriverWait(driver, 20).until(
        EC.element_to_be_clickable((By.XPATH, "//ul[@role='menu' and @aria-label='File']//li[text()='Open']"))
    )
    code_menu_item.click()

    time.sleep(20)  


    # Capture a screenshot
    driver.get_screenshot_as_file('screenshot.png')

finally:
    # Close the browser to ensure the session is properly terminated
    if driver:
        driver.quit()
