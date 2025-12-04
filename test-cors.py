#!/usr/bin/env python3
"""
CORS Test Script
Tests CORS configuration for the API server
"""

import requests
import json

API_URL = "https://textbook-hvqhdgkym-mathnjs-projects.vercel.app"
ORIGIN = "https://mathnj.github.io"

print("=" * 70)
print("CORS Configuration Test")
print("=" * 70)
print(f"API URL: {API_URL}")
print(f"Origin: {ORIGIN}")
print("=" * 70)

# Test 1: Health Check (GET)
print("\n1. Testing GET / (health check)...")
try:
    response = requests.get(f"{API_URL}/", headers={
        "Origin": ORIGIN
    })
    print(f"   Status: {response.status_code}")
    print(f"   Response: {response.json()}")
    print("   CORS Headers:")
    for header, value in response.headers.items():
        if 'access-control' in header.lower() or 'origin' in header.lower():
            print(f"     - {header}: {value}")
except Exception as e:
    print(f"   [X] Error: {e}")

# Test 2: Preflight Request (OPTIONS)
print("\n2. Testing OPTIONS /chat (preflight request)...")
try:
    response = requests.options(f"{API_URL}/chat", headers={
        "Origin": ORIGIN,
        "Access-Control-Request-Method": "POST",
        "Access-Control-Request-Headers": "content-type"
    })
    print(f"   Status: {response.status_code}")
    print("   CORS Headers:")
    for header, value in response.headers.items():
        if 'access-control' in header.lower():
            print(f"     - {header}: {value}")

    # Check required CORS headers
    required_headers = [
        "access-control-allow-origin",
        "access-control-allow-methods",
        "access-control-allow-headers"
    ]
    missing = []
    for header in required_headers:
        if header not in [h.lower() for h in response.headers.keys()]:
            missing.append(header)

    if missing:
        print(f"   [X] Missing headers: {missing}")
    else:
        print("   [OK] All required CORS headers present")

except Exception as e:
    print(f"   [X] Error: {e}")

# Test 3: Actual POST Request
print("\n3. Testing POST /chat (actual request)...")
try:
    response = requests.post(
        f"{API_URL}/chat",
        headers={
            "Origin": ORIGIN,
            "Content-Type": "application/json"
        },
        json={
            "message": "Hello, test message",
            "use_rag": False
        },
        timeout=30
    )
    print(f"   Status: {response.status_code}")
    print("   CORS Headers:")
    for header, value in response.headers.items():
        if 'access-control' in header.lower():
            print(f"     - {header}: {value}")

    if response.status_code == 200:
        print(f"   [OK] Response: {response.json()}")
    else:
        print(f"   Response body: {response.text[:200]}")

except Exception as e:
    print(f"   [X] Error: {e}")

# Test 4: Check environment variables endpoint
print("\n4. Testing GET /api/test (environment check)...")
try:
    response = requests.get(f"{API_URL}/api/test", headers={
        "Origin": ORIGIN
    })
    print(f"   Status: {response.status_code}")
    if response.status_code == 200:
        data = response.json()
        print("   Environment Variables:")
        for key, value in data.get("environment_variables", {}).items():
            print(f"     - {key}: {value}")
except Exception as e:
    print(f"   [X] Error: {e}")

print("\n" + "=" * 70)
print("Test Complete")
print("=" * 70)
