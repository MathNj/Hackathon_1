"""
Vercel Serverless Function for FastAPI
Entry point for Vercel deployment
"""

from main import app

# Export app for Vercel
# Vercel will automatically handle the ASGI interface
