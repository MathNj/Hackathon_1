"""
FastAPI Backend for Physical AI Textbook Platform
Main application entry point with CORS, routers, and health checks
"""

import os
from typing import List, Optional
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from dotenv import load_dotenv
import logging
import traceback
from agent import get_agent

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI Textbook API",
    description="Backend API for RAG chatbot, authentication, and personalization",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc"
)

# CORS Configuration
# Allow requests from GitHub Pages and localhost
CORS_ORIGINS = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")

app.add_middleware(
    CORSMiddleware,
    allow_origins=CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Health check endpoint
@app.get("/")
async def root():
    """Root endpoint - health check"""
    return {
        "status": "healthy",
        "message": "Physical AI Textbook API is running",
        "version": "1.0.0"
    }

@app.get("/api/health")
async def health_check():
    """Detailed health check with service status"""
    return {
        "status": "healthy",
        "services": {
            "api": "running",
            "database": "not configured",  # Will be updated when Neon is added
            "qdrant": "not configured",     # Will be updated when Qdrant is added
            "openai": "configured" if os.getenv("GEMINI_API_KEY") else "not configured"
        },
        "environment": os.getenv("ENVIRONMENT", "development")
    }

# Pydantic models for request/response
class ChatMessage(BaseModel):
    """Single message in chat history"""
    role: str  # "user" or "assistant"
    content: str

class ChatRequest(BaseModel):
    """Chat request with message and optional history"""
    message: str
    history: Optional[List[ChatMessage]] = []

class ChatResponse(BaseModel):
    """Chat response with answer and sources"""
    response: str
    sources: Optional[List[dict]] = []
    model: Optional[str] = None
    usage: Optional[dict] = None

class PersonalizeRequest(BaseModel):
    """Personalization request with content and user preferences"""
    content: str
    hardware_bg: str
    skill_level: str

class PersonalizeResponse(BaseModel):
    """Personalized content response"""
    personalized_content: str
    model: Optional[str] = None

class SearchRequest(BaseModel):
    """Search request for vector-based content search"""
    query: str
    limit: int = 5

class SearchResult(BaseModel):
    """Individual search result"""
    title: str
    content: str
    url: str
    score: float

class SearchResponse(BaseModel):
    """Search response with multiple results"""
    results: List[SearchResult]
    query: str
    total: int

# API Routes

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Chat endpoint for RAG-powered question answering

    Args:
        request: ChatRequest with message and optional history

    Returns:
        ChatResponse with answer and source citations
    """
    try:
        # Get the agent instance
        agent = get_agent()

        # Generate response using RAG
        result = await agent.generate_response(
            query=request.message,
            use_rag=True
        )

        if not result.get("success", False):
            raise HTTPException(
                status_code=500,
                detail=result.get("error", "Failed to generate response")
            )

        return ChatResponse(
            response=result["answer"],
            sources=result.get("sources", []),
            model=result.get("model"),
            usage=result.get("usage")
        )

    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}", exc_info=True)
        print("=" * 60)
        print("❌ DETAILED ERROR in /chat endpoint:")
        print("=" * 60)
        print(f"Error Type: {type(e).__name__}")
        print(f"Error Message: {str(e)}")
        print(f"Request Message: {request.message[:100]}...")
        print("\nFull Traceback:")
        traceback.print_exc()
        print("=" * 60)
        raise HTTPException(status_code=500, detail=f"{type(e).__name__}: {str(e)}")

@app.post("/personalize", response_model=PersonalizeResponse)
async def personalize(request: PersonalizeRequest):
    """
    Personalize content based on user's hardware and skill level
    Uses direct Gemini API call with custom system prompt

    Args:
        request: PersonalizeRequest with content, hardware_bg, and skill_level

    Returns:
        PersonalizeResponse with personalized content
    """
    try:
        # Get the agent instance to access the client
        agent = get_agent()

        # Build system prompt with the specified format
        system_prompt = f"""Rewrite the following robotics technical documentation for a {request.skill_level} level student using {request.hardware_bg}. Adjust complexity and implementation details accordingly."""

        # Call Gemini 2.5 Flash API directly using client.chat.completions.create
        response = await agent.client.chat.completions.create(
            model="gemini-2.5-flash",  # Gemini 2.5 Flash model
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": request.content}
            ],
            max_tokens=2000,
            temperature=0.7
        )

        # Extract the personalized content
        personalized_content = response.choices[0].message.content

        return PersonalizeResponse(
            personalized_content=personalized_content,
            model=response.model
        )

    except Exception as e:
        logger.error(f"Error in personalize endpoint: {str(e)}", exc_info=True)
        print("=" * 60)
        print("❌ DETAILED ERROR in /personalize endpoint:")
        print("=" * 60)
        print(f"Error Type: {type(e).__name__}")
        print(f"Error Message: {str(e)}")
        print(f"Content Length: {len(request.content)} chars")
        print(f"Skill Level: {request.skill_level}")
        print(f"Hardware: {request.hardware_bg}")
        print("\nFull Traceback:")
        traceback.print_exc()
        print("=" * 60)
        raise HTTPException(status_code=500, detail=f"{type(e).__name__}: {str(e)}")

@app.post("/search", response_model=SearchResponse)
async def search(request: SearchRequest):
    """
    Fast vector-based search endpoint (no LLM generation)

    Searches the Qdrant vector database for relevant content chunks.
    Returns results in <200ms for instant search experience.

    Args:
        request: SearchRequest with query and optional limit

    Returns:
        SearchResponse with matched content chunks and scores
    """
    try:
        # Get the agent instance to access Qdrant client
        agent = get_agent()

        # Generate embedding for the search query
        logger.info(f"Generating embedding for search query: {request.query[:50]}...")
        query_embedding = await agent.embed_text(request.query)

        # Search Qdrant (bypasses LLM for speed)
        logger.info(f"Searching Qdrant with limit={request.limit}")
        search_results = agent.qdrant_client.search(
            collection_name=agent.collection_name,
            query_vector=query_embedding,
            limit=request.limit
        )

        # Format results
        results = []
        for result in search_results:
            # Extract metadata from payload
            filename = result.payload.get("filename", "Unknown")
            header = result.payload.get("header", "")
            text = result.payload.get("text", "")

            # Generate URL based on filename
            # Example: "module-1-ros2/basics.md" -> "/docs/en/module-1-ros2/basics"
            url = f"/docs/en/{filename.replace('.md', '')}" if filename != "Unknown" else "#"

            # Create title from header or filename
            title = header if header else filename.replace(".md", "").replace("-", " ").title()

            # Truncate content for preview (max 200 chars)
            content_preview = text[:200] + "..." if len(text) > 200 else text

            results.append(SearchResult(
                title=title,
                content=content_preview,
                url=url,
                score=result.score
            ))

        logger.info(f"Search completed: found {len(results)} results")

        return SearchResponse(
            results=results,
            query=request.query,
            total=len(results)
        )

    except Exception as e:
        logger.error(f"Error in search endpoint: {str(e)}", exc_info=True)
        print("=" * 60)
        print("❌ DETAILED ERROR in /search endpoint:")
        print("=" * 60)
        print(f"Error Type: {type(e).__name__}")
        print(f"Error Message: {str(e)}")
        print(f"Query: {request.query}")
        print(f"Limit: {request.limit}")
        print("\nFull Traceback:")
        traceback.print_exc()
        print("=" * 60)
        raise HTTPException(status_code=500, detail=f"{type(e).__name__}: {str(e)}")

@app.get("/api/test")
async def test_endpoint():
    """Test endpoint to verify API is working"""
    return {
        "message": "API is working correctly",
        "environment_variables": {
            "GEMINI_API_KEY": "configured" if os.getenv("GEMINI_API_KEY") else "not set",
            "OPENAI_API_BASE": os.getenv("OPENAI_API_BASE", "not set"),
            "DATABASE_URL": "configured" if os.getenv("DATABASE_URL") else "not set",
            "QDRANT_URL": os.getenv("QDRANT_URL", "not set")
        }
    }

# TODO: Add /api/auth/* endpoints for authentication (Phase 4)
# TODO: Add /api/profile/* endpoints for user profiles (Phase 5)

# Error handlers
@app.exception_handler(HTTPException)
async def http_exception_handler(request, exc):
    """Custom HTTP exception handler"""
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": exc.detail,
            "status_code": exc.status_code
        }
    )

@app.exception_handler(Exception)
async def general_exception_handler(request, exc):
    """General exception handler for unexpected errors"""
    logger.error(f"Unexpected error: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": "Internal server error",
            "message": str(exc) if os.getenv("DEBUG") == "true" else "An unexpected error occurred"
        }
    )

# Startup event
@app.on_event("startup")
async def startup_event():
    """Run on application startup"""
    logger.info("Physical AI Textbook API starting up...")
    logger.info(f"Environment: {os.getenv('ENVIRONMENT', 'development')}")
    logger.info(f"CORS Origins: {CORS_ORIGINS}")

    # DEBUG: Verify critical environment variables
    print("=" * 60)
    print("🔍 DEBUG: LLM & Embedding Configuration")
    print("=" * 60)
    print(f"DEBUG: OPENAI_API_BASE (Gemini) = {os.getenv('OPENAI_API_BASE')}")
    print(f"DEBUG: GEMINI_API_KEY = {bool(os.getenv('GEMINI_API_KEY'))} (for Gemini 2.5 Flash)")
    if os.getenv("GEMINI_API_KEY"):
        key_preview = os.getenv("GEMINI_API_KEY")[:10] + "..." if len(os.getenv("GEMINI_API_KEY", "")) > 10 else "invalid"
        print(f"       Preview: {key_preview}")
    else:
        print("       ❌ WARNING: Not set - Chat will not work")

    print(f"DEBUG: OPENAI_API_KEY = {bool(os.getenv('OPENAI_API_KEY'))} (for embeddings)")
    if os.getenv("OPENAI_API_KEY"):
        key_preview = os.getenv("OPENAI_API_KEY")[:10] + "..." if len(os.getenv("OPENAI_API_KEY", "")) > 10 else "invalid"
        print(f"       Preview: {key_preview}")
    else:
        print("       ❌ WARNING: Not set - Search/RAG will not work")

    print(f"DEBUG: DATABASE_URL = {os.getenv('DATABASE_URL')[:30]}..." if os.getenv('DATABASE_URL') else "Not set")
    print(f"DEBUG: QDRANT_URL = {os.getenv('QDRANT_URL', 'Not set')}")
    print("=" * 60)
    print("📊 Model Configuration:")
    print("  - Chat: Gemini 2.5 Flash (via Google Generative AI)")
    print("  - Embeddings: text-embedding-3-small (via OpenAI)")
    print("  - Vector DB: Qdrant")
    print("=" * 60)

    if not os.getenv("GEMINI_API_KEY"):
        logger.warning("GEMINI_API_KEY not set - Chat functionality will not work")

    if not os.getenv("OPENAI_API_KEY"):
        logger.warning("OPENAI_API_KEY not set - Embeddings/Search/RAG will not work")

    logger.info("Startup complete - API ready to accept requests")

# Shutdown event
@app.on_event("shutdown")
async def shutdown_event():
    """Run on application shutdown"""
    logger.info("Physical AI Textbook API shutting down...")
    # Close database connections, cleanup resources, etc.
    logger.info("Shutdown complete")

if __name__ == "__main__":
    import uvicorn

    # Get configuration from environment
    host = os.getenv("API_HOST", "0.0.0.0")
    port = int(os.getenv("API_PORT", "8000"))
    reload = os.getenv("DEBUG", "true").lower() == "true"

    logger.info(f"Starting server on {host}:{port}")

    uvicorn.run(
        "main:app",
        host=host,
        port=port,
        reload=reload,
        log_level="info"
    )
