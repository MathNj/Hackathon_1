# Document Ingestion Scripts

## Overview

This directory contains scripts for ingesting textbook content into the Qdrant vector database for RAG (Retrieval-Augmented Generation) functionality.

## Scripts

### `ingest.py` - Document Ingestion Pipeline

Recursively reads markdown files, chunks content, generates embeddings, and stores in Qdrant.

#### Features

- **Multiple Chunking Strategies**:
  - `headers`: Split by markdown headers (## or ###) - **Recommended**
  - `size`: Fixed-size chunks with overlap (500 characters default)

- **Automatic Embedding Generation**: Uses OpenAI text-embedding-3-small (1536 dimensions)

- **Qdrant Integration**: Stores vectors with metadata (filename, header, text)

- **Error Handling**: Graceful handling of failed embeddings and missing files

#### Prerequisites

1. **Environment Variables** (in `.env` file):
   ```bash
   GEMINI_API_KEY=your_api_key_here
   OPENAI_API_BASE=https://generativelanguage.googleapis.com/v1beta/openai/
   QDRANT_URL=http://localhost:6333  # Or cloud URL
   QDRANT_API_KEY=  # Optional for cloud
   ```

2. **Qdrant Running**:
   ```bash
   # Option 1: Docker (Recommended for local development)
   docker run -p 6333:6333 qdrant/qdrant

   # Option 2: Cloud (Qdrant Cloud free tier)
   # Sign up at https://qdrant.tech/
   # Update QDRANT_URL and QDRANT_API_KEY in .env
   ```

3. **Dependencies Installed**:
   ```bash
   cd api
   pip install -r requirements.txt
   ```

#### Usage

##### Basic Usage (Headers Strategy)

```bash
cd api
python scripts/ingest.py
```

This will:
1. Read all `.md` files from `web/docs/`
2. Split by headers (## or ###)
3. Generate embeddings for each chunk
4. Upload to Qdrant collection `robotics_textbook`

##### Custom Documentation Directory

```bash
python scripts/ingest.py --docs-dir path/to/docs
```

##### Size-Based Chunking

```bash
python scripts/ingest.py --strategy size
```

#### Command-Line Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--docs-dir` | str | `web/docs` | Path to documentation directory |
| `--strategy` | str | `headers` | Chunking strategy: `headers` or `size` |

#### What Happens During Ingestion

1. **Collection Setup**:
   - Checks if `robotics_textbook` collection exists
   - Prompts to recreate if it exists (deletes old data)
   - Creates collection with 1536-dimensional vectors (cosine similarity)

2. **File Reading**:
   - Recursively finds all `.md` files
   - Skips Urdu translations (`.ur.md` files)
   - Logs each file read

3. **Chunking**:
   - **Headers strategy**: Splits at `##` and `###` headers
   - **Size strategy**: 500-character chunks with 50-character overlap
   - Filters out chunks < 50 characters

4. **Embedding Generation**:
   - Calls OpenAI API (Gemini backend) for each chunk
   - Progress logged every 10 embeddings
   - Failed embeddings are skipped (logged)

5. **Qdrant Upload**:
   - Uploads in batches of 100 points
   - Each point includes:
     - `id`: Sequential integer
     - `vector`: 1536-dimensional embedding
     - `payload`: `{text, header, filename}`

#### Expected Output

```
2025-12-03 18:30:00 - __main__ - INFO - Starting document ingestion...
2025-12-03 18:30:00 - __main__ - INFO - Collection 'robotics_textbook' already exists
Recreate collection? This will delete all existing data. (yes/no): yes
2025-12-03 18:30:01 - __main__ - INFO - Deleted existing collection 'robotics_textbook'
2025-12-03 18:30:01 - __main__ - INFO - Created collection 'robotics_textbook'
2025-12-03 18:30:01 - __main__ - INFO - Read file: module-0-setup/intro.md
2025-12-03 18:30:01 - __main__ - INFO - Read file: module-1-nervous-system/intro.md
...
2025-12-03 18:30:02 - __main__ - INFO - Found 6 markdown files
2025-12-03 18:30:02 - __main__ - INFO - Chunked module-0-setup/intro.md: 8 chunks
...
2025-12-03 18:30:02 - __main__ - INFO - Total chunks created: 52
2025-12-03 18:30:02 - __main__ - INFO - Generating embeddings...
2025-12-03 18:30:05 - __main__ - INFO - Generated 10/52 embeddings
2025-12-03 18:30:08 - __main__ - INFO - Generated 20/52 embeddings
...
2025-12-03 18:30:15 - __main__ - INFO - Successfully embedded 52/52 chunks
2025-12-03 18:30:15 - __main__ - INFO - Uploading 52 chunks to Qdrant...
2025-12-03 18:30:15 - __main__ - INFO - Uploaded batch 1/1
2025-12-03 18:30:15 - __main__ - INFO - Successfully uploaded 52 points to Qdrant
2025-12-03 18:30:15 - __main__ - INFO - Ingestion complete!
```

#### Verification

After ingestion, verify the data:

```bash
# Check collection info
curl http://localhost:6333/collections/robotics_textbook

# Sample search (using Qdrant API)
curl -X POST http://localhost:6333/collections/robotics_textbook/points/search \
  -H "Content-Type: application/json" \
  -d '{"vector": [0.1, 0.2, ...], "limit": 3}'
```

Or use the health check endpoint:

```bash
# Start the API
python api/main.py

# Check health
curl http://localhost:8000/api/health
```

Should show:
```json
{
  "status": "healthy",
  "qdrant": "healthy",
  "collection_exists": true
}
```

#### Troubleshooting

##### Error: "GEMINI_API_KEY not set in environment"

**Solution**: Make sure `.env` file exists in project root with your API key:
```bash
GEMINI_API_KEY=your_actual_key_here
```

##### Error: "Connection refused" to Qdrant

**Solution**: Start Qdrant:
```bash
docker run -p 6333:6333 qdrant/qdrant
```

Or update `QDRANT_URL` in `.env` to your cloud instance.

##### Error: "No documents found to ingest"

**Solution**: Check that `web/docs/` directory exists and contains `.md` files:
```bash
ls web/docs/*/*.md
```

##### Error: Rate limit exceeded

**Solution**: The script processes embeddings sequentially. If you hit rate limits:
1. Add a delay between embedding calls (modify `embed_chunks` method)
2. Use a paid Gemini API tier with higher limits
3. Process in smaller batches

##### Warning: "Failed to embed chunk X"

This is usually non-critical. The script continues with successful chunks. Check:
- API key validity
- Network connectivity
- Chunk content (might contain special characters)

#### Performance

**Expected Times** (for 6 intro modules, ~50 chunks):
- File reading: <1 second
- Chunking: <1 second
- Embedding generation: 10-15 seconds (depends on API latency)
- Qdrant upload: <1 second

**Total**: ~15-20 seconds

#### Best Practices

1. **Incremental Updates**: If you add new content, recreate the collection to avoid duplicates
2. **Strategy Selection**: Use `headers` for structured content (chapters, sections), `size` for unstructured text
3. **Monitoring**: Watch API costs (each embedding call = 1 API request)
4. **Backup**: Qdrant data is persistent in Docker volumes - back up if needed

#### Next Steps

After ingestion:
1. Test RAG search: `python api/agent.py`
2. Start API server: `python api/main.py`
3. Query via `/chat` endpoint:
   ```bash
   curl -X POST http://localhost:8000/chat \
     -H "Content-Type: application/json" \
     -d '{"message": "What is ROS 2?"}'
   ```

## Future Enhancements

- [ ] Incremental ingestion (detect new/modified files only)
- [ ] Parallel embedding generation (faster processing)
- [ ] Metadata enrichment (module number, difficulty level, tags)
- [ ] Support for code snippets (special handling)
- [ ] Multilingual support (ingest Urdu translations separately)
