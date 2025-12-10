---
id: 72
title: Create Technical Plan for RAG Chatbot
stage: plan
date: 2025-12-07
surface: agent
model: gemini-pro
feature: online-book
branch: main
user: "[user]"
command: Generate the full technical implementation plan for the new RAG Chatbot feature as specified in the updated '.specify/memory/spec.md' and append it to the end of '.specify/memory/plan.md'. The plan must be structured into three main component stages:

1.  Vectorization Pipeline: Steps to install dependencies, scrape the Docusaurus book content (MDX/Markdown), chunk the text, use OpenAI embeddings, and upload vectors to a Qdrant Cloud collection.
2.  FastAPI Backend Service: Define the server setup, including the RAG endpoint that handles the incoming query and optional selected text, performs the Qdrant search, and uses the OpenAI Agent/LLM to generate the final response.
3.  Docusaurus Frontend: Design the required React Component to integrate the ChatKit UI, implement the logic for capturing user-selected text, and manage API communication.

Ensure all details comply with the Context7 rule in '.specify/memory/constitution.md'.
labels: [plan, rag-chatbot, feature]
links:
  spec: book-project-specify/.specify/memory/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - book-project-specify/.specify/memory/plan.md
tests:
 - "none"
---

## Prompt

Generate the full technical implementation plan for the new RAG Chatbot feature as specified in the updated '.specify/memory/spec.md' and append it to the end of '.specify/memory/plan.md'. The plan must be structured into three main component stages:

1.  Vectorization Pipeline: Steps to install dependencies, scrape the Docusaurus book content (MDX/Markdown), chunk the text, use OpenAI embeddings, and upload vectors to a Qdrant Cloud collection.
2.  FastAPI Backend Service: Define the server setup, including the RAG endpoint that handles the incoming query and optional selected text, performs the Qdrant search, and uses the OpenAI Agent/LLM to generate the final response.
3.  Docusaurus Frontend: Design the required React Component to integrate the ChatKit UI, implement the logic for capturing user-selected text, and manage API communication.

Ensure all details comply with the Context7 rule in '.specify/memory/constitution.md'.

## Response snapshot

# Technical Implementation Plan: RAG Chatbot

## 1. Scope and Dependencies

### In Scope
-   **Vectorization Pipeline:** A script to process all `.md` and `.mdx` files in the `docs/` directory, chunk them, generate embeddings via OpenAI, and store them in a Qdrant Cloud collection.
-   **FastAPI Backend:** A Python-based API with a single endpoint (`/chat`) that accepts a user query and optional selected text, retrieves context from Qdrant, and returns a response from an OpenAI Agent.
-   **Docusaurus Frontend:** A React component that provides the chat UI, captures selected text from the document, and communicates with the FastAPI backend.

### Out of Scope
-   User authentication for the chatbot.
-   Storing chat history.
-   Advanced UI features beyond the ChatKit SDK's standard capabilities.

### Dependencies
-   **Qdrant Cloud:** A free-tier account and collection must be set up. API Key and URL will be required.
-   **OpenAI API:** An API key is required for both embedding generation and the chat agent.
-   **Docusaurus Project:** A functional Docusaurus site.

## 2. Key Decisions and Rationale

### Vectorization Pipeline
-   **Technology:** A Python script (`scripts/vectorize.py`) will be created. Python is chosen for its robust data processing libraries (e.g., `langchain` for chunking) and the availability of official SDKs from OpenAI and Qdrant.
-   **Content Source:** The script will target the `docs/` directory, recursively scanning for `.md` and `.mdx` files. This ensures all book content is indexed.
-   **Chunking Strategy:** Text will be split using a `RecursiveCharacterTextSplitter` (from LangChain) with a chunk size of 1000 characters and an overlap of 200. This strategy is effective for Markdown as it attempts to split on semantic boundaries first (paragraphs, lines).
-   **Embedding Model:** `text-embedding-3-small` from OpenAI will be used. It provides a strong balance of performance and cost-effectiveness.
-   **Vector Store:** Qdrant Cloud is used as specified. A single collection named `book_content` will store the vectors and their corresponding text payloads.

### FastAPI Backend
-   **Framework:** FastAPI is chosen for its high performance, automatic OpenAPI documentation, and Python type-hinting features, making it ideal for building a robust API quickly.
-   **Endpoint (`/api/chat`):** A single POST endpoint is sufficient. It will accept a Pydantic model containing `query: str` and `selected_text: Optional[str]`.
-   **RAG Logic:**
    1.  If `selected_text` is provided, it will be prepended to the `query` to form a combined search prompt for Qdrant. This directly implements the "Contextual Mode" requirement.
    2.  The combined prompt will be used to search the Qdrant collection for the top 3 most relevant text chunks.
    3.  The retrieved chunks (context) and the original query will be formatted into a prompt for the OpenAI Chat Completions API (`gpt-4o`).
    4.  The prompt will explicitly instruct the model to answer the query based *only* on the provided context and to cite the source document (metadata stored in the vector).
-   **CORS:** The FastAPI server will be configured with CORS (Cross-Origin Resource Sharing) middleware to allow requests from the Docusaurus development server and the deployed site.

### Docusaurus Frontend
-   **Technology:** A new React component, `Chatbot`, will be created in `src/theme/`. Placing it in the theme folder allows it to be swizzled and integrated easily. It will use the **OpenAI ChatKit SDK** for the UI.
-   **Text Selection Logic:** The component will use the browser's `window.getSelection()` API to capture highlighted text. A `mouseup` event listener on the main document body will trigger the text capture. The captured text will be stored in the component's state.
-   **API Communication:** The `fetch` API will be used to send POST requests to the FastAPI backend. The component will manage loading, error, and response states.
-   **Integration:** The `Chatbot` component will be added to the `Root.tsx` file in the theme, ensuring it is present on all pages of the Docusaurus site.

## 3. Non-Functional Requirements

-   **Constitutional Compliance:** All frontend code will be written in TypeScript and adhere to the project's React and style conventions.
-   **Context7 Principle:** The prompts sent to the OpenAI agent will be engineered to be version-aware by including context from the retrieved documents, which are themselves generated under the Context7 principle. This ensures the agent's responses are grounded in the correct version of the book's content.
-   **Security:** API keys (OpenAI, Qdrant) will not be hardcoded. The FastAPI server will load them from environment variables. The frontend will not have access to these keys.

## 4. Evaluation and Validation (Definition of Done)

-   [ ] The `scripts/vectorize.py` script successfully populates the Qdrant collection with data from the `docs/` directory.
-   [ ] The FastAPI server runs, and the `/api/chat` endpoint returns a valid response when tested with tools like `curl` or Postman.
-   [ ] The Chatbot UI component appears on the Docusaurus site.
-   [ ] When a user asks a question, the chatbot provides an answer based on the book's content.
-   [ ] When a user selects text and then asks a question, the chatbot's answer reflects the context of the selected text.
-   [ ] The chatbot's answers include citations pointing to the source document.

## 5. Implementation Steps

### Stage 1: Vectorization Pipeline
1.  **Setup:**
    -   Add `fastapi`, `uvicorn`, `qdrant-client`, `openai`, and `langchain` to a `requirements.txt` file for the backend.
    -   Create a Qdrant Cloud account and a new collection named `book_content`.
    -   Set up a `.env` file to store `QDRANT_URL`, `QDRANT_API_KEY`, and `OPENAI_API_KEY`.
2.  **Create `scripts/vectorize.py`:**
    -   Implement logic to find all `.md`/`.mdx` files in `docs/`.
    -   Use `RecursiveCharacterTextSplitter` to chunk the files.
    -   Use the `openai` library to generate embeddings for each chunk.
    -   Use the `qdrant-client` to upload the vectors and their text content as payload to the `book_content` collection. Include the source file path as metadata.

### Stage 2: FastAPI Backend
1.  **Create `api/main.py`:**
    -   Initialize a FastAPI app.
    -   Add CORS middleware.
    -   Create Pydantic models for the request (`query`, `selected_text`) and response.
    -   Implement the `/chat` endpoint.
    -   Inside the endpoint:
        a.  Initialize Qdrant and OpenAI clients.
        b.  Combine `query` and `selected_text` for searching.
        c.  Perform a similarity search on Qdrant.
        d.  Construct a detailed prompt for the OpenAI Chat Completions API, including the retrieved context.
        e.  Return the model's response.

### Stage 3: Docusaurus Frontend
1.  **Create `src/theme/Chatbot/index.tsx`:**
    -   Use the OpenAI ChatKit SDK to scaffold the chat UI.
    -   Add a `useEffect` hook to register a `mouseup` event listener on `document.body`.
    -   The event handler should call `window.getSelection().toString()` and update the component's state with the selected text.
    -   Implement the function that sends the user's message and the selected text to the `/api/chat` backend endpoint.
2.  **Integrate the Component:**
    -   Modify `src/theme/Root.tsx` to include the `<Chatbot />` component, wrapping the main application layout. This ensures it's available on every page.
3.  **Final Validation:**
    -   Run the FastAPI server (`uvicorn api.main:app --reload`).
    -   Run the Docusaurus dev server (`npm run start`).
    -   Test all acceptance criteria outlined in the "Evaluation and Validation" section.

## Outcome

- ✅ Impact: The technical implementation plan for the RAG Chatbot feature has been created and appended to the plan file.
- 🧪 Tests: No tests were run.
- 📁 Files: Modified 1 file.
- 🔁 Next prompts: Begin implementation of the vectorization pipeline.
- 🧠 Reflection: The plan covers all the user's requirements and provides a clear path for implementation.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
