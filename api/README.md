# Physical AI Textbook - Backend API

FastAPI backend for the Physical AI & Humanoid Robotics Textbook platform.

## Features

- **RAG Chatbot**: OpenAI Agents SDK with Gemini API backend
- **CORS Support**: Configured for localhost and GitHub Pages
- **Health Checks**: API status monitoring
- **Environment Management**: Secure credential handling with .env

## Quick Start

### 1. Install Dependencies

```bash
cd api
pip install -r requirements.txt
```

### 2. Configure Environment

Copy the `.env` file from the project root and fill in your API keys:

```bash
# Required
GEMINI_API_KEY=your_gemini_api_key_here
OPENAI_API_BASE=https://generativelanguage.googleapis.com/v1beta/openai/

# Optional (for development)
DEBUG=true
API_HOST=0.0.0.0
API_PORT=8000
```

### 3. Run the Server

```bash
# Development mode (with auto-reload)
python main.py

# Production mode
uvicorn main:app --host 0.0.0.0 --port 8000
```

The API will be available at:
- **API**: http://localhost:8000
- **Docs**: http://localhost:8000/api/docs
- **Health**: http://localhost:8000/api/health

## API Endpoints

### Core Endpoints

- `GET /` - Root health check
- `GET /api/health` - Detailed service status
- `GET /api/test` - Test endpoint with environment info

### RAG Endpoints (Phase 3)

- `POST /api/rag/query` - Ask a question to the textbook
- `POST /api/rag/feedback` - Submit feedback on responses

### Auth Endpoints (Phase 4)

- `POST /api/auth/signup` - Create new user account
- `POST /api/auth/login` - User login
- `GET /api/auth/me` - Get current user profile

### Profile Endpoints (Phase 5)

- `GET /api/profile/me` - Get user profile
- `PUT /api/profile/me` - Update user profile
- `GET /api/profile/export` - Export user data (GDPR)

## Project Structure

```
api/
├── main.py              # FastAPI application entry point
├── agent.py             # RAG agent with AsyncOpenAI
├── requirements.txt     # Python dependencies
├── README.md           # This file
│
├── routers/            # API route handlers (Phase 3+)
│   ├── rag.py         # RAG chatbot routes
│   ├── auth.py        # Authentication routes
│   └── profile.py     # User profile routes
│
├── services/          # Business logic (Phase 3+)
│   ├── qdrant_client.py    # Vector database client
│   ├── embeddings.py       # Text embedding service
│   └── auth_service.py     # Authentication logic
│
├── models/            # Data models (Phase 3+)
│   ├── user.py       # User SQLAlchemy model
│   ├── query.py      # RAG query model
│   └── progress.py   # Learning progress model
│
└── tests/            # Unit and integration tests
    ├── test_main.py
    └── test_agent.py
```

## Testing the Agent

Test the RAG agent directly:

```bash
python agent.py
```

This will:
1. Check API connectivity
2. Send a test query ("What is ROS 2?")
3. Display the response

## Development

### Code Formatting

```bash
# Format code
black .

# Lint code
ruff check .

# Type check
mypy .
```

### Running Tests

```bash
pytest
```

## Environment Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `GEMINI_API_KEY` | Yes | - | Your Gemini API key |
| `OPENAI_API_BASE` | Yes | Gemini URL | OpenAI-compatible base URL |
| `DATABASE_URL` | No | - | Neon Postgres connection string |
| `QDRANT_URL` | No | localhost:6333 | Qdrant vector database URL |
| `QDRANT_API_KEY` | No | - | Qdrant API key (for cloud) |
| `DEBUG` | No | false | Enable debug mode |
| `ENVIRONMENT` | No | development | Environment name |
| `CORS_ORIGINS` | No | localhost:3000 | Allowed CORS origins (comma-separated) |

## Troubleshooting

### Issue: "GEMINI_API_KEY not found"

**Solution**: Make sure the `.env` file is in the project root and contains your API key:

```bash
GEMINI_API_KEY=your_key_here
```

### Issue: "Connection refused" on localhost:8000

**Solution**: Make sure the server is running:

```bash
python main.py
```

### Issue: CORS errors from frontend

**Solution**: Add your frontend URL to `CORS_ORIGINS` in `.env`:

```bash
CORS_ORIGINS=http://localhost:3000,https://your-github-pages-url.github.io
```

## Next Steps

- **Phase 3**: Add Qdrant integration for vector search
- **Phase 4**: Implement Better-Auth for user authentication
- **Phase 5**: Add personalization endpoints

## License

Part of the Physical AI & Humanoid Robotics Textbook project.
