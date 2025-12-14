import google.generativeai as genai
from fastembed import TextEmbedding
from fastapi import FastAPI
from pydantic import BaseModel
from typing import Optional
from qdrant_client import QdrantClient
from fastapi.middleware.cors import CORSMiddleware
import logging

from .config import settings

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ChatRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    response: str

app = FastAPI()

origins = [
    "http://localhost:3000",
    "http://localhost",
    "https://physical-ai-humanoid-robotics-taupe.vercel.app",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

qdrant_client = QdrantClient(
    url=settings.QDRANT_URL,
    api_key=settings.QDRANT_API_KEY,
    timeout=60,
)
embedding_model = TextEmbedding()
genai.configure(api_key=settings.GEMINI_API_KEY)

BASE_CONTEXT = (
    "Book title: Physical AI & Humanoid Robotics (The Guide to Modern Agentic Development). "
    "Scope: open-source book covering hardware, software, agents, and robotics."
)

@app.get("/")
def read_root():
    return {"Hello": "World"}

@app.post("/api/chat", response_model=ChatResponse)
def chat(request: ChatRequest):
    # Quick friendly reply for simple greetings without hitting LLM or Qdrant
    if request.query.strip().lower() in {"hi", "hello", "hey", "hi!", "hello!", "hey!"}:
        return ChatResponse(response="Hi! I'm your assistant for the Physical AI & Humanoid Robotics book. Ask me anything about the book's content.")

    search_query = request.query
    if request.selected_text:
        search_query = f"{request.selected_text}\n\n{request.query}"

    try:
        embedding_result = list(embedding_model.embed([search_query]))
        if not embedding_result:
            return ChatResponse(response="Sorry, there was an error processing your request.")
        embedding = embedding_result[0].tolist()
    except Exception as e:
        logger.error(f"Error generating embedding: {e}", exc_info=True)
        return ChatResponse(response="Sorry, there was an error processing your request.")

    try:
        search_result = qdrant_client.search(
            collection_name="book_content",
            query_vector=embedding,
            limit=8,
        )
        context = []
        for hit in search_result:
            if not hit.payload:
                continue
            text = hit.payload.get("text", "")
            if text:
                context.append(text)
    except Exception as e:
        logger.error(f"Error retrieving context from Qdrant: {e}", exc_info=True)
        return ChatResponse(response="Sorry, there was an error retrieving information.")

    # Ensure we always provide some grounding context
    if not context:
        context = [BASE_CONTEXT]
    else:
        context.append(BASE_CONTEXT)

    context_str = "\n\n---\n\n".join(filter(None, context))
    prompt = f"""
You are a helpful assistant for the book "Physical AI & Humanoid Robotics" (The Guide to Modern Agentic Development).
Answer ONLY from the context below; if the answer is not there, say you don't know.

Context:
{context_str}

Question:
{request.query}
"""

    try:
        model = genai.GenerativeModel('gemini-2.5-flash')
        response = model.generate_content(prompt)
        
        if not response or not response.candidates:
            logger.error("No response candidates from Gemini")
            return ChatResponse(response="Sorry, I couldn't generate a response.")
        
        return ChatResponse(response=response.text)
    except Exception as e:
        logger.error(f"Error generating response from Gemini LLM: {e}", exc_info=True)
        return ChatResponse(response=f"Sorry, there was an error generating a response: {str(e)}")
