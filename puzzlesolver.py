import requests
import os
import time
import base64
from openai import OpenAI
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from selenium.common.exceptions import TimeoutException
from urllib.parse import urljoin, urlparse

# --- Configuration ---
# 1. Set your OpenAI API Key as an environment variable:
#    export OPENAI_API_KEY="your-api-key-here"
#    Or paste it directly below (NOT RECOMMENDED for production)
# WARNING: Do not commit API keys to version control!
api_key = os.getenv("OPENAI_API_KEY", "")  # Get from environment variable

# 2. The page to scrape
page_url = "https://drift-tech.github.io/hack-quiz/"

# 3. The ID of the image tag to find
tag_id = "randomImage"

# 4. The question to ask OpenAI
your_question = "solve the puzzle in this image. your answer should be 1, 2, or 3. only output one number and nothing else"
# --- End Configuration ---


def get_mime_type_from_url(url):
    """Helper function to guess the image MIME type from its URL."""
    # Get the file extension from the URL path
    path = urlparse(url).path
    ext = os.path.splitext(path)[1].lower()
    
    if ext == ".png":
        return "image/png"
    elif ext in [".jpg", ".jpeg"]:
        return "image/jpeg"
    elif ext == ".gif":
        return "image/gif"
    elif ext == ".webp":
        return "image/webp"
    else:
        # Default to png if unknown, as it's common
        print(f"Warning: Unknown image extension '{ext}'. Defaulting to 'image/png'.")
        return "image/png"


def find_and_solve_image(page_url, tag_id, api_key, question):
    """
    Loads a dynamic page, finds the current image URL,
    fetches the image data in memory, and sends it to OpenAI.
    """
    
    print("Starting browser...")
    # This automatically manages the chromedriver
    driver = webdriver.Chrome()
    
    try:
        # === PART 1: SELENIUM TO FIND THE IMAGE URL ===
        print(f"Loading page: {page_url}...")
        driver.get(page_url)

        print(f"Waiting for JavaScript to pick an image and set 'src'...")
        wait = WebDriverWait(driver, 10)
        
        # 1. Wait for the tag with that ID to even exist
        image_tag = wait.until(
            EC.presence_of_element_located((By.ID, tag_id))
        )
        
        # 2. Wait *again* until that tag's 'src' attribute is not empty
        wait.until(
            lambda d: image_tag.get_attribute("src") and image_tag.get_attribute("src") != ""
        )

        # 3. Get the 'src' URL from the live, rendered page
        image_url = image_tag.get_attribute('src')
        if not image_url:
            print("Error: Waited, but the tag still has no 'src' attribute.")
            return

        print(f"Found dynamic image URL: {image_url}")

        # === PART 2: FETCH IMAGE IN MEMORY ===
        # Handle relative URLs (e.g., /images/pic.png)
        full_image_url = urljoin(page_url, image_url)
        
        print(f"Fetching image data from: {full_image_url}...")
        image_response = requests.get(full_image_url)
        image_response.raise_for_status()
        # image_response.content now holds the raw bytes of the image

        # === PART 3: PREPARE AND SEND TO OPENAI ===
        
        # 1. Encode the raw image bytes (from memory) to base64
        base64_image = base64.b64encode(image_response.content).decode('utf-8')
        
        # 2. Determine the image's MIME type (e.g., image/png)
        mime_type = get_mime_type_from_url(full_image_url)
        
        # 3. Initialize OpenAI client
        client = OpenAI(api_key=api_key)

        print(f"Image loaded in memory. Asking OpenAI: '{question}'...")
        
        # 4. Send to OpenAI
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": question},
                        {
                            "type": "image_url",
                            "image_url": {
                                # Use the dynamic MIME type and base64 data
                                "url": f"data:{mime_type};base64,{base64_image}"
                            },
                        },
                    ],
                }
            ],
            max_tokens=300
        )

        answer = response.choices[0].message.content
        print("\n--- Answer from OpenAI ---")
        print(answer)

    except TimeoutException:
        print(f"Error: Page loaded, but the image with id='{tag_id}' did not get a 'src' attribute after 10 seconds.")
    except requests.exceptions.RequestException as e:
        print(f"Error fetching image data: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # 7. Always close the browser window
        print("Closing browser.")
        driver.quit()

# --- Run the function ---
if __name__ == "__main__":
    if not api_key or "sk-" not in api_key:
         print("Error: Please paste your OpenAI API Key into the 'api_key' variable.")
    else:
        find_and_solve_image(page_url, tag_id, api_key, your_question)