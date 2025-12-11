from .main import app

# Vercel detects this file as the serverless function entrypoint.
# `app` is the FastAPI ASGI application imported from main.py.
