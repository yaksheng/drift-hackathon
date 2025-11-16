#!/usr/bin/env python3
"""Simple test to verify puzzle solver works"""

import sys
import requests
import base64
from openai import OpenAI

# Configuration
# Set your OpenAI API Key as an environment variable:
# export OPENAI_API_KEY="your-api-key-here"
import os
api_key = os.getenv("OPENAI_API_KEY", "")  # Get from environment variable
image_url = "https://drift-hack.s3.ap-south-1.amazonaws.com/3.png"
question = "solve the puzzle in this image. your answer should be 1, 2, or 3. only output one number and nothing else"

print("="*60)
print("PUZZLE SOLVER TEST")
print("="*60)
print(f"Image URL: {image_url}")
print(f"Question: {question}")
print()

try:
    # Download image
    print("Step 1: Downloading image...")
    image_response = requests.get(image_url)
    image_response.raise_for_status()
    print(f"✅ Image downloaded ({len(image_response.content)} bytes)")
    
    # Encode to base64
    print("Step 2: Encoding image to base64...")
    base64_image = base64.b64encode(image_response.content).decode('utf-8')
    print("✅ Image encoded")
    
    # Send to OpenAI
    print("Step 3: Sending to OpenAI...")
    client = OpenAI(api_key=api_key)
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
                            "url": f"data:image/png;base64,{base64_image}"
                        },
                    },
                ],
            }
        ],
        max_tokens=300
    )
    
    answer = response.choices[0].message.content.strip()
    
    print()
    print("="*60)
    print("ANSWER FROM OPENAI:")
    print(answer)
    print("="*60)
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

