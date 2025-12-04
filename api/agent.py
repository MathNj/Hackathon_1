"""
OpenAI Agent Client Setup for RAG Chatbot
Uses AsyncOpenAI with Gemini API compatibility
"""

import os
from typing import Optional, List, Dict, Any
from openai import AsyncOpenAI
from dotenv import load_dotenv
import logging
import traceback
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)

class RAGAgent:
    """
    RAG Agent for Physical AI Textbook
    Uses OpenAI Agents SDK with Gemini API backend
    """

    def __init__(self):
        """Initialize the RAG agent with AsyncOpenAI client and Qdrant

        Configuration:
        - Chat completions: Gemini 2.5 Flash (gemini-2.5-flash)
        - Embeddings: OpenAI (text-embedding-3-small)
        - Vector DB: Qdrant
        """
        # Get API configuration from environment
        self.api_key = os.getenv("GEMINI_API_KEY")
        self.base_url = os.getenv("OPENAI_API_BASE", "https://generativelanguage.googleapis.com/v1beta/openai/")

        if not self.api_key:
            raise ValueError("GEMINI_API_KEY not found in environment variables")

        # Initialize AsyncOpenAI client with Gemini compatibility
        # Uses Gemini 2.5 Flash for chat, OpenAI embeddings via Gemini proxy
        self.client = AsyncOpenAI(
            api_key=self.api_key,
            base_url=self.base_url
        )

        # Initialize Qdrant client for vector search
        qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        self.qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key
        )
        self.collection_name = "robotics_textbook"

        logger.info("RAG Agent initialized with Gemini API backend and Qdrant")

    async def generate_response(
        self,
        query: str,
        context: Optional[List[str]] = None,
        max_tokens: int = 500,
        temperature: float = 0.7,
        use_rag: bool = True
    ) -> Dict[str, Any]:
        """
        Generate a response to a user query using the RAG pipeline

        Args:
            query: User's question
            context: Retrieved context chunks (if provided, skips RAG search)
            max_tokens: Maximum tokens in response
            temperature: Sampling temperature (0.0 = deterministic, 1.0 = creative)
            use_rag: Whether to use RAG search (default: True)

        Returns:
            Dictionary with response text and metadata
        """
        try:
            # If context not provided and RAG is enabled, search textbook
            sources = []
            if context is None and use_rag:
                search_results = await self.search_textbook(query, top_k=3)
                if search_results:
                    context = [result["text"] for result in search_results]
                    sources = [
                        {
                            "filename": result["filename"],
                            "header": result["header"],
                            "score": result["score"]
                        }
                        for result in search_results
                    ]

            # Build the system prompt
            system_prompt = self._build_system_prompt()

            # Build the user message with context
            user_message = self._build_user_message(query, context)

            # Call the API with Gemini 2.5 Flash
            response = await self.client.chat.completions.create(
                model="gemini-2.5-flash",  # Gemini 2.5 Flash model
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message}
                ],
                max_tokens=max_tokens,
                temperature=temperature
            )

            # Extract response
            answer = response.choices[0].message.content

            return {
                "success": True,
                "answer": answer,
                "sources": sources,
                "model": response.model,
                "usage": {
                    "prompt_tokens": response.usage.prompt_tokens,
                    "completion_tokens": response.usage.completion_tokens,
                    "total_tokens": response.usage.total_tokens
                }
            }

        except Exception as e:
            logger.error(f"Error generating response: {str(e)}", exc_info=True)
            print("=" * 60)
            print("❌ DETAILED ERROR in generate_response:")
            print("=" * 60)
            print(f"Error Type: {type(e).__name__}")
            print(f"Error Message: {str(e)}")
            print("\nFull Traceback:")
            traceback.print_exc()
            print("=" * 60)
            return {
                "success": False,
                "error": str(e),
                "error_type": type(e).__name__,
                "answer": "I apologize, but I encountered an error processing your question. Please try again."
            }

    def _build_system_prompt(self) -> str:
        """Build the system prompt for the RAG agent"""
        return """You are an AI assistant for the Physical AI & Humanoid Robotics Textbook.

Your role is to help students learn about:
- ROS 2 (Robot Operating System)
- Robotics simulation (Gazebo, Unity, Isaac Sim)
- Visual SLAM and navigation (Nav2)
- Vision-Language-Action (VLA) models
- Physical AI and embodied intelligence

Guidelines:
1. Answer questions based ONLY on the textbook content provided in the context
2. If the context doesn't contain relevant information, say "I need more context — this topic isn't covered in the textbook"
3. Keep responses concise (max 300 words)
4. Include citations to textbook sections when possible (e.g., "See Module 1: ROS 2 Basics")
5. Use technical accuracy - this is an advanced robotics course
6. If asked about hardware requirements, reference RTX 4070 Ti and Jetson Orin
7. For code examples, use Python with ROS 2 unless otherwise specified

Remember: You are a helpful teaching assistant, not a general-purpose chatbot."""

    def _build_user_message(self, query: str, context: Optional[List[str]] = None) -> str:
        """Build the user message with query and context"""
        if context and len(context) > 0:
            # Format context chunks
            context_str = "\n\n".join([f"[Context {i+1}]:\n{chunk}" for i, chunk in enumerate(context)])

            return f"""Based on the following context from the textbook, please answer the student's question:

{context_str}

Student's Question: {query}

Remember to cite the relevant module/section and keep your answer concise."""
        else:
            # No context available (for Phase 1 testing without Qdrant)
            return f"""Student's Question: {query}

Note: Full RAG context retrieval is not yet configured. Please provide a helpful response based on general Physical AI and robotics knowledge, but remind the student that detailed answers will be available once the vector database is set up."""

    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embeddings for text using OpenAI's embedding model
        Will be used for Qdrant indexing in Phase 3

        Args:
            text: Text to embed

        Returns:
            List of floats representing the embedding vector
        """
        try:
            response = await self.client.embeddings.create(
                model="text-embedding-3-small",
                input=text
            )

            return response.data[0].embedding

        except Exception as e:
            logger.error(f"Error generating embedding: {str(e)}", exc_info=True)
            print("=" * 60)
            print("❌ DETAILED ERROR in embed_text:")
            print("=" * 60)
            print(f"Error Type: {type(e).__name__}")
            print(f"Error Message: {str(e)}")
            print("\nFull Traceback:")
            traceback.print_exc()
            print("=" * 60)
            raise

    async def search_textbook(self, query: str, top_k: int = 3) -> List[Dict[str, Any]]:
        """
        Search the textbook using cosine similarity on Qdrant

        Args:
            query: User's search query
            top_k: Number of results to return (default: 3)

        Returns:
            List of top matching chunks with metadata
        """
        try:
            # Generate embedding for the query
            query_embedding = await self.embed_text(query)

            # Search Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k
            )

            # Format results
            results = []
            for result in search_results:
                results.append({
                    "text": result.payload["text"],
                    "header": result.payload["header"],
                    "filename": result.payload["filename"],
                    "score": result.score
                })

            logger.info(f"Found {len(results)} results for query: {query[:50]}...")
            return results

        except Exception as e:
            logger.error(f"Error searching textbook: {str(e)}", exc_info=True)
            print("=" * 60)
            print("❌ DETAILED ERROR in search_textbook:")
            print("=" * 60)
            print(f"Error Type: {type(e).__name__}")
            print(f"Error Message: {str(e)}")
            print(f"Query: {query[:100]}...")
            print("\nFull Traceback:")
            traceback.print_exc()
            print("=" * 60)
            return []

    async def health_check(self) -> Dict[str, Any]:
        """Check if the API connection is working"""
        try:
            # Make a simple API call to test connectivity with Gemini 2.5 Flash
            response = await self.client.chat.completions.create(
                model="gemini-2.5-flash",
                messages=[{"role": "user", "content": "Hello"}],
                max_tokens=10
            )

            # Check Qdrant connection
            try:
                collections = self.qdrant_client.get_collections()
                qdrant_status = "healthy"
                collection_exists = any(c.name == self.collection_name for c in collections.collections)
            except Exception as e:
                qdrant_status = f"unhealthy: {str(e)}"
                collection_exists = False

            return {
                "status": "healthy",
                "model": response.model,
                "api_base": self.base_url,
                "qdrant": qdrant_status,
                "collection_exists": collection_exists
            }

        except Exception as e:
            logger.error(f"Health check failed: {str(e)}", exc_info=True)
            print("=" * 60)
            print("❌ DETAILED ERROR in health_check:")
            print("=" * 60)
            print(f"Error Type: {type(e).__name__}")
            print(f"Error Message: {str(e)}")
            print(f"API Base URL: {self.base_url}")
            print(f"API Key Present: {bool(self.api_key)}")
            print("\nFull Traceback:")
            traceback.print_exc()
            print("=" * 60)
            return {
                "status": "unhealthy",
                "error": str(e),
                "error_type": type(e).__name__
            }

# Global agent instance (singleton pattern)
_agent_instance: Optional[RAGAgent] = None

def get_agent() -> RAGAgent:
    """Get or create the global RAG agent instance"""
    global _agent_instance

    if _agent_instance is None:
        _agent_instance = RAGAgent()

    return _agent_instance

# Example usage (for testing)
async def main():
    """Test the agent"""
    agent = get_agent()

    # Test health check
    health = await agent.health_check()
    print(f"Health check: {health}")

    # Test query
    response = await agent.generate_response(
        query="What is ROS 2?",
        context=None
    )
    print(f"Response: {response}")

if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
