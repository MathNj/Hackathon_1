"""
Document Ingestion Script for RAG Pipeline
Recursively reads markdown files, chunks content, generates embeddings, and stores in Qdrant
"""

import os
import sys
from pathlib import Path
from typing import List, Dict, Any
import asyncio
from dotenv import load_dotenv
import logging
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from openai import AsyncOpenAI

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent))

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class DocumentIngester:
    """Handles ingestion of markdown documents into Qdrant vector database"""

    def __init__(self):
        """Initialize the ingester with OpenAI and Qdrant clients"""
        # OpenAI client for embeddings
        self.openai_client = AsyncOpenAI(
            api_key=os.getenv("GEMINI_API_KEY"),
            base_url=os.getenv("OPENAI_API_BASE", "https://generativelanguage.googleapis.com/v1beta/openai/")
        )

        # Qdrant client
        qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        # Determine if using Qdrant Cloud (https URL) or local instance
        is_cloud = qdrant_url.startswith("https://")

        if is_cloud:
            # For Qdrant Cloud, use URL without port and prefer REST API
            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                prefer_grpc=False,
                https=True,
                port=None  # Don't add port for cloud
            )
        else:
            # For local Qdrant, use standard connection
            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key
            )

        self.collection_name = "robotics_textbook"
        self.embedding_dimension = 1536  # text-embedding-3-small dimension

        logger.info(f"Initialized DocumentIngester with Qdrant at {qdrant_url}")

    def setup_collection(self):
        """Create or recreate the Qdrant collection"""
        try:
            # Check if collection exists
            collections = self.qdrant_client.get_collections().collections
            collection_exists = any(c.name == self.collection_name for c in collections)

            if collection_exists:
                logger.info(f"Collection '{self.collection_name}' already exists")
                # Optionally recreate it
                response = input("Recreate collection? This will delete all existing data. (yes/no): ")
                if response.lower() == 'yes':
                    self.qdrant_client.delete_collection(self.collection_name)
                    logger.info(f"Deleted existing collection '{self.collection_name}'")
                else:
                    logger.info("Using existing collection")
                    return

            # Create collection
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.embedding_dimension,
                    distance=Distance.COSINE
                )
            )
            logger.info(f"Created collection '{self.collection_name}'")

        except Exception as e:
            logger.error(f"Error setting up collection: {str(e)}", exc_info=True)
            raise

    def read_markdown_files(self, docs_dir: str) -> List[Dict[str, Any]]:
        """
        Recursively read all .md files from the docs directory

        Args:
            docs_dir: Path to the documentation directory

        Returns:
            List of dictionaries with file content and metadata
        """
        markdown_files = []
        docs_path = Path(docs_dir)

        if not docs_path.exists():
            logger.error(f"Documentation directory not found: {docs_dir}")
            return []

        # Find all .md files (excluding .ur.md Urdu files)
        for md_file in docs_path.rglob("*.md"):
            # Skip Urdu translations
            if md_file.name.endswith(".ur.md"):
                continue

            try:
                with open(md_file, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Extract relative path for metadata
                relative_path = md_file.relative_to(docs_path)

                markdown_files.append({
                    "filename": str(relative_path),
                    "full_path": str(md_file),
                    "content": content
                })

                logger.info(f"Read file: {relative_path}")

            except Exception as e:
                logger.error(f"Error reading {md_file}: {str(e)}")

        logger.info(f"Found {len(markdown_files)} markdown files")
        return markdown_files

    def chunk_by_headers(self, content: str, filename: str) -> List[Dict[str, Any]]:
        """
        Split content by markdown headers (## or ###)

        Args:
            content: Markdown content
            filename: Source filename for metadata

        Returns:
            List of chunks with metadata
        """
        chunks = []
        lines = content.split('\n')

        current_header = "Introduction"
        current_chunk = []

        for line in lines:
            # Check for headers (## or ###)
            if line.startswith('## ') or line.startswith('### '):
                # Save previous chunk if it exists
                if current_chunk:
                    chunk_text = '\n'.join(current_chunk).strip()
                    if len(chunk_text) > 50:  # Minimum chunk size
                        chunks.append({
                            "text": chunk_text,
                            "header": current_header,
                            "filename": filename
                        })

                # Start new chunk
                current_header = line.lstrip('#').strip()
                current_chunk = [line]
            else:
                current_chunk.append(line)

        # Add final chunk
        if current_chunk:
            chunk_text = '\n'.join(current_chunk).strip()
            if len(chunk_text) > 50:
                chunks.append({
                    "text": chunk_text,
                    "header": current_header,
                    "filename": filename
                })

        return chunks

    def chunk_by_size(self, content: str, filename: str, chunk_size: int = 500, overlap: int = 50) -> List[Dict[str, Any]]:
        """
        Split content into fixed-size chunks with overlap

        Args:
            content: Text content
            filename: Source filename
            chunk_size: Maximum characters per chunk
            overlap: Overlap between chunks

        Returns:
            List of chunks with metadata
        """
        chunks = []
        start = 0

        while start < len(content):
            end = start + chunk_size
            chunk_text = content[start:end].strip()

            if len(chunk_text) > 50:  # Minimum chunk size
                chunks.append({
                    "text": chunk_text,
                    "header": f"Chunk at position {start}",
                    "filename": filename
                })

            start = end - overlap

        return chunks

    def chunk_documents(self, documents: List[Dict[str, Any]], strategy: str = "headers") -> List[Dict[str, Any]]:
        """
        Chunk all documents using the specified strategy

        Args:
            documents: List of document dictionaries
            strategy: "headers" or "size"

        Returns:
            List of all chunks
        """
        all_chunks = []

        for doc in documents:
            if strategy == "headers":
                chunks = self.chunk_by_headers(doc["content"], doc["filename"])
            else:
                chunks = self.chunk_by_size(doc["content"], doc["filename"])

            all_chunks.extend(chunks)
            logger.info(f"Chunked {doc['filename']}: {len(chunks)} chunks")

        logger.info(f"Total chunks created: {len(all_chunks)}")
        return all_chunks

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a text chunk

        Args:
            text: Text to embed

        Returns:
            Embedding vector
        """
        try:
            response = await self.openai_client.embeddings.create(
                model="text-embedding-3-small",
                input=text
            )
            return response.data[0].embedding

        except Exception as e:
            logger.error(f"Error generating embedding: {str(e)}")
            raise

    async def embed_chunks(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Generate embeddings for all chunks

        Args:
            chunks: List of text chunks

        Returns:
            Chunks with embeddings added
        """
        logger.info("Generating embeddings...")

        for i, chunk in enumerate(chunks):
            try:
                embedding = await self.generate_embedding(chunk["text"])
                chunk["embedding"] = embedding

                if (i + 1) % 10 == 0:
                    logger.info(f"Generated {i + 1}/{len(chunks)} embeddings")

            except Exception as e:
                logger.error(f"Error embedding chunk {i}: {str(e)}")
                chunk["embedding"] = None

        # Filter out chunks with failed embeddings
        valid_chunks = [c for c in chunks if c.get("embedding") is not None]
        logger.info(f"Successfully embedded {len(valid_chunks)}/{len(chunks)} chunks")

        return valid_chunks

    def upsert_to_qdrant(self, chunks: List[Dict[str, Any]]):
        """
        Upload chunks with embeddings to Qdrant

        Args:
            chunks: Chunks with embeddings
        """
        logger.info(f"Uploading {len(chunks)} chunks to Qdrant...")

        points = []
        for i, chunk in enumerate(chunks):
            point = PointStruct(
                id=i,
                vector=chunk["embedding"],
                payload={
                    "text": chunk["text"],
                    "header": chunk["header"],
                    "filename": chunk["filename"]
                }
            )
            points.append(point)

        # Upload in batches
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=batch
            )
            logger.info(f"Uploaded batch {i // batch_size + 1}/{(len(points) + batch_size - 1) // batch_size}")

        logger.info(f"Successfully uploaded {len(points)} points to Qdrant")

    async def ingest(self, docs_dir: str, strategy: str = "headers"):
        """
        Main ingestion pipeline

        Args:
            docs_dir: Path to documentation directory
            strategy: Chunking strategy ("headers" or "size")
        """
        logger.info("Starting document ingestion...")

        # Setup collection
        self.setup_collection()

        # Read markdown files
        documents = self.read_markdown_files(docs_dir)
        if not documents:
            logger.error("No documents found to ingest")
            return

        # Chunk documents
        chunks = self.chunk_documents(documents, strategy=strategy)

        # Generate embeddings
        chunks_with_embeddings = await self.embed_chunks(chunks)

        # Upload to Qdrant
        self.upsert_to_qdrant(chunks_with_embeddings)

        logger.info("Ingestion complete!")


async def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description="Ingest documentation into Qdrant")
    parser.add_argument(
        "--docs-dir",
        type=str,
        default="web/docs",
        help="Path to documentation directory (default: web/docs)"
    )
    parser.add_argument(
        "--strategy",
        type=str,
        choices=["headers", "size"],
        default="headers",
        help="Chunking strategy: 'headers' or 'size' (default: headers)"
    )

    args = parser.parse_args()

    # Validate environment
    if not os.getenv("GEMINI_API_KEY"):
        logger.error("GEMINI_API_KEY not set in environment")
        sys.exit(1)

    # Run ingestion
    ingester = DocumentIngester()
    await ingester.ingest(args.docs_dir, args.strategy)


if __name__ == "__main__":
    asyncio.run(main())
