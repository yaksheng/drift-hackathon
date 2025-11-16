import requests
import os
import time
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from selenium.common.exceptions import TimeoutException
from urllib.parse import urljoin # To handle relative paths

# -----------------------------------------------------------------
# The page you want to scrape
url = "https://drift-tech.github.io/hack-quiz/"
# The ID of the image tag
tag_id = "randomImage"
# -----------------------------------------------------------------


def download_current_dynamic_image(page_url, tag_id):
    """
    Loads a dynamic page, waits for JavaScript to set a variable image 'src',
    and then downloads that specific image.
    """
    
    # Set up the Selenium Chrome driver
    # This will open a new Chrome window
    print("Starting browser...")
    driver = webdriver.Chrome()
    
    try:
        # 1. Load the page in the automated browser
        # The JavaScript will run and pick a random image
        print(f"Loading page: {page_url}...")
        driver.get(page_url)

        # 2. Wait for the image's 'src' attribute to be added
        print(f"Waiting for JavaScript to pick an image and set the 'src'...")
        
        # We wait up to 10 seconds
        wait = WebDriverWait(driver, 10)
        
        # This is the key:
        # First, wait for the tag with that ID to even exist
        image_tag = wait.until(
            EC.presence_of_element_located((By.ID, tag_id))
        )
        
        # Second, wait *again* until that tag's 'src' attribute
        # is not empty. This proves the JS has run.
        wait.until(
            lambda d: image_tag.get_attribute("src") and image_tag.get_attribute("src") != ""
        )

        # 3. Get the 'src' attribute *after* JavaScript has run
        image_url = image_tag.get_attribute('src')
        
        if not image_url:
            print("Error: Waited, but the tag still has no 'src' attribute.")
            return

        print(f"Found dynamic image URL: {image_url}")

        # 4. Join the base URL with the image URL
        # This handles cases where the src is relative (e.g., "/images/pic.png")
        full_image_url = urljoin(page_url, image_url)
        print(f"Downloading from: {full_image_url}")

        # 5. Download the image using 'requests'
        image_response = requests.get(full_image_url)
        image_response.raise_for_status()

        # 6. Save the image
        filename = os.path.basename(full_image_url.split('?')[0])
        if not filename:
            filename = f"{tag_id}_downloaded.png"

        with open(filename, 'wb') as f:
            f.write(image_response.content)

        print(f"\n✅ Success! Image saved as: {filename}")

    except TimeoutException:
        print(f"Error: Page loaded, but the image with id='{tag_id}' did not get a 'src' attribute after 10 seconds.")
    except requests.exceptions.RequestException as e:
        print(f"Error downloading image: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # 7. Always close the browser window
        print("Closing browser.")
        driver.quit()

# --- Run the function ---
if __name__ == "__main__":
    download_current_dynamic_image(url, tag_id)