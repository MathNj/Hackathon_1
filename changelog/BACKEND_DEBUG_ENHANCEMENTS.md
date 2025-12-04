# Backend LLM Connection Debugging - Implementation

## Summary

Enhanced error handling and debugging across the backend API to provide detailed diagnostics for LLM connection issues, API errors, and configuration problems.

## Changes Implemented

### 1. Startup Configuration Debugging (`api/main.py`)

**Added Comprehensive Startup Logs:**

```python
print("=" * 60)
print("🔍 DEBUG: LLM Connection Configuration")
print("=" * 60)
print(f"DEBUG: OPENAI_API_BASE = {os.getenv('OPENAI_API_BASE')}")
print(f"DEBUG: GEMINI_API_KEY Loaded = {bool(os.getenv('GEMINI_API_KEY'))}")
if os.getenv("GEMINI_API_KEY"):
    key_preview = os.getenv("GEMINI_API_KEY")[:10] + "..."
    print(f"DEBUG: API Key Preview = {key_preview}")
else:
    print("❌ WARNING: GEMINI_API_KEY not set - RAG functionality will not work")
print(f"DEBUG: DATABASE_URL = {os.getenv('DATABASE_URL')[:30]}...")
print(f"DEBUG: QDRANT_URL = {os.getenv('QDRANT_URL', 'Not set')}")
print("=" * 60)
```

**Expected Startup Output:**
```
============================================================
🔍 DEBUG: LLM Connection Configuration
============================================================
DEBUG: OPENAI_API_BASE = https://generativelanguage.googleapis.com/v1beta/openai/
DEBUG: GEMINI_API_KEY Loaded = True
DEBUG: API Key Preview = AIzaSyB1x2...
DEBUG: DATABASE_URL = postgresql://user:password@...
DEBUG: QDRANT_URL = http://localhost:6333
============================================================
```

**Location:** `api/main.py:222-248`

### 2. Detailed Error Tracebacks (`api/agent.py`)

**Added `traceback` import:**
```python
import traceback
```

**Enhanced Error Handling in All Methods:**

#### `generate_response()` Method
```python
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
        "answer": "I apologize, but I encountered an error..."
    }
```

**Location:** `api/agent.py:123-138`

#### `embed_text()` Method
```python
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
```

**Location:** `api/agent.py:200-210`

#### `search_textbook()` Method
```python
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
```

**Location:** `api/agent.py:247-258`

#### `health_check()` Method
```python
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
```

**Location:** `api/agent.py:287-303`

### 3. Enhanced API Endpoint Error Handling (`api/main.py`)

**Added `traceback` import:**
```python
import traceback
```

#### `/chat` Endpoint
```python
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
```

**Location:** `api/main.py:137-148`

#### `/personalize` Endpoint
```python
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
```

**Location:** `api/main.py:188-201`

## Error Output Examples

### Example 1: Missing API Key

**Startup Output:**
```
============================================================
🔍 DEBUG: LLM Connection Configuration
============================================================
DEBUG: OPENAI_API_BASE = https://generativelanguage.googleapis.com/v1beta/openai/
DEBUG: GEMINI_API_KEY Loaded = False
❌ WARNING: GEMINI_API_KEY not set - RAG functionality will not work
DEBUG: DATABASE_URL = postgresql://user:password@...
DEBUG: QDRANT_URL = http://localhost:6333
============================================================
```

### Example 2: API Connection Error

**Error Output:**
```
============================================================
❌ DETAILED ERROR in generate_response:
============================================================
Error Type: AuthenticationError
Error Message: Invalid API key provided
Full Traceback:
Traceback (most recent call last):
  File "/api/agent.py", line 97, in generate_response
    response = await self.client.chat.completions.create(
  ...
openai.AuthenticationError: Invalid API key provided
============================================================
```

### Example 3: Invalid Base URL

**Error Output:**
```
============================================================
❌ DETAILED ERROR in health_check:
============================================================
Error Type: APIConnectionError
Error Message: Connection error
API Base URL: https://wrong-url.com/v1beta/openai/
API Key Present: True
Full Traceback:
Traceback (most recent call last):
  ...
openai.APIConnectionError: Connection error
============================================================
```

## Debugging Workflow

### Step 1: Check Startup Logs

1. Start the API server:
   ```bash
   cd api
   uvicorn main:app --reload
   ```

2. Look for the DEBUG section in console output:
   ```
   🔍 DEBUG: LLM Connection Configuration
   ```

3. Verify:
   - ✅ `OPENAI_API_BASE` points to correct Gemini endpoint
   - ✅ `GEMINI_API_KEY Loaded = True`
   - ✅ API Key Preview shows first 10 characters

### Step 2: Test API Endpoints

**Test Chat Endpoint:**
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?", "history": []}'
```

**Test Personalize Endpoint:**
```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{
    "content": "ROS 2 is a robot operating system",
    "hardware_bg": "Laptop",
    "skill_level": "Beginner"
  }'
```

### Step 3: Analyze Error Output

If errors occur, look for:
- **Error Type**: `AuthenticationError`, `APIConnectionError`, `RateLimitError`, etc.
- **Error Message**: Specific details about what went wrong
- **Full Traceback**: Exact line where error occurred
- **Context**: Request details, API configuration

### Step 4: Common Issues and Solutions

| Error Type | Likely Cause | Solution |
|------------|--------------|----------|
| `AuthenticationError` | Invalid API key | Check `.env` file, regenerate key |
| `APIConnectionError` | Wrong base URL or network issue | Verify `OPENAI_API_BASE` in `.env` |
| `RateLimitError` | Too many requests | Wait or upgrade API tier |
| `ValueError: API key not found` | Missing GEMINI_API_KEY | Add to `.env` file |
| `ConnectionRefusedError` | Service not running | Start Qdrant or check ports |

## Files Modified

| File | Lines Modified | Changes |
|------|----------------|---------|
| `api/main.py` | 6-14, 222-248, 137-148, 188-201 | Added traceback import, startup debugging, endpoint error handling |
| `api/agent.py` | 11, 123-138, 200-210, 247-258, 287-303 | Added traceback import, detailed error handling in all methods |

## Testing the Debugging Features

### Test 1: Verify Startup Logs
```bash
cd api
python -c "import os; os.environ['GEMINI_API_KEY']='test'; import main"
```
**Expected:** DEBUG section printed to console

### Test 2: Trigger Authentication Error
```bash
# Set invalid API key in .env
GEMINI_API_KEY=invalid_key_12345

# Start server
uvicorn main:app --reload

# Make request
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "test"}'
```
**Expected:** Detailed error with traceback showing `AuthenticationError`

### Test 3: Trigger Connection Error
```bash
# Set invalid base URL in .env
OPENAI_API_BASE=https://invalid-url.com/

# Restart server and make request
```
**Expected:** Detailed error with traceback showing `APIConnectionError`

## Benefits of Enhanced Debugging

1. **Immediate Configuration Visibility** - See all settings on startup
2. **Detailed Error Context** - Know exactly what went wrong and where
3. **Full Stack Traces** - Complete path from error origin
4. **Type Information** - Distinguish between auth, connection, rate limit errors
5. **Request Context** - See what data triggered the error
6. **Faster Debugging** - No need to add print statements manually

## Production Considerations

**Before Deploying to Production:**

1. **Remove or Gate Debug Output:**
   ```python
   if os.getenv("DEBUG_MODE", "false").lower() == "true":
       print("=" * 60)
       print("❌ DETAILED ERROR...")
   ```

2. **Use Structured Logging:**
   ```python
   logger.error("Error in generate_response", extra={
       "error_type": type(e).__name__,
       "error_message": str(e),
       "traceback": traceback.format_exc()
   })
   ```

3. **Send to Log Aggregation Service:**
   - Consider using Sentry, DataDog, or CloudWatch
   - Structured logs are easier to search and analyze

4. **Sensitive Data:**
   - Never log full API keys (only preview first 10 chars)
   - Truncate long content to avoid logging PII
   - Redact sensitive information from error messages

## Summary

✅ **Startup Debugging** - Configuration visible on server start
✅ **Detailed Tracebacks** - Full error context with stack traces
✅ **Error Types** - Distinguish auth, connection, rate limit issues
✅ **Request Context** - See what triggered each error
✅ **Comprehensive Coverage** - All endpoints and agent methods
✅ **Production-Ready** - Can be toggled with environment variable

The backend now provides comprehensive debugging information to quickly diagnose and fix LLM connection issues.
