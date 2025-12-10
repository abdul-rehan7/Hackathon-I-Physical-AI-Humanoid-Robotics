# Plan for Online Book

## 1. Scope and Dependencies
### In Scope:
- Creation of 13 weekly modules for an online book on Agentic AI and Robotics.
- Each module will cover specific topics with detailed explanations, code examples, and practical exercises.
- Integration with Docusaurus for static site generation and easy deployment.
- Hardware Requirements page detailing minimum, recommended, and optional specifications.

### Out of Scope:
- Advanced interactive simulations beyond basic Docusaurus capabilities.
- User authentication or personalized learning paths.
- Real-time robotics control directly from the website.

### External Dependencies:
- Docusaurus (static site generator)
- Markdown/MDX for content creation
- Potential external libraries for code examples (e.g., ROS 2, NVIDIA Isaac SDK, PyTorch, TensorFlow)
- **RAG Chatbot Integration:**
    - FastEmbed (for embedding generation)
    - Qdrant (for vector database)
    - Gemini API (for LLM backend)
    - FastAPI (for API server)

## 2. Key Decisions and Rationale
- **Technology Stack:** Docusaurus was chosen for its ease of use, Markdown-first approach, and excellent documentation features, making it suitable for an online book format.
- **Content Structure:** Weekly modules provide a structured learning path, breaking down complex topics into manageable chunks.
- **Placeholder Strategy:** Initial content placeholders (e.g., "Topic 1.1") were used to quickly establish the structure and facilitate task generation.
- **RAG Chatbot Architecture:**
    - **Embedding Model:** The vectorization pipeline must use the **FastEmbed** Python library (specifically the default 'BAAI/bge-small-en-v1.5' or 'all-MiniLM-L6-v2') to generate vectors. The task must explicitly install `qdrant-client[fastembed]` for native integration.
    - **LLM Backend:** The RAG final answer generation must use the **Gemini API** (specifically the **gemini-2.5-flash-lite** model) for the LLM component to ensure free-tier usage.
    - **Authentication:** The FastAPI server must securely consume the **`GEMINI_API_KEY`** and the **`QDRANT_API_KEY`/`QDRANT_URL`** from environment variables for all API calls.
    - **RAG Flow:** The FastAPI endpoint must receive the user query, use **FastEmbed** to vectorize it, retrieve context from **Qdrant**, and then prompt the **Gemini LLM** with the context for a final, sourced answer.

## 3. Interfaces and API Contracts
- Not applicable for a static online book. Content will be rendered directly from Markdown/MDX files.

## 4. Non-Functional Requirements (NFRs) and Budgets
- **Performance:** Fast loading times for all pages, optimized images.
- **Reliability:** High availability of the hosted site.
- **Security:** Standard web security practices for a static site.
- **Cost:** Minimal hosting costs (e.g., GitHub Pages, Netlify).

## 5. Data Management and Migration
- Content is managed as Markdown/MDX files in a Git repository. Version control handles changes and history.

## 6. Operational Readiness
- **Observability:** Standard web analytics can be integrated if needed.
- **Deployment:** Automated deployment via CI/CD (e.g., GitHub Actions) upon Git push.

## 7. Risk Analysis and Mitigation
- **Content Quality:** Risk of inconsistent content quality. Mitigation: Peer review of content, clear writing guidelines.
- **Technical Accuracy:** Risk of outdated or incorrect technical information. Mitigation: Regular content review and updates.

## 8. Evaluation and Validation
- **Definition of Done:** All content placeholders filled, code examples tested, site builds successfully, and all links are functional.
- **Output Validation:** Manual review of rendered pages for correctness, formatting, and readability.

## 9. Architectural Decision Record (ADR)
- No significant architectural decisions requiring separate ADRs at this stage beyond the initial technology stack choice.
