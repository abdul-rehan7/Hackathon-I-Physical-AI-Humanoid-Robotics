---
id: 71
title: Explain How to Use FastAPI Docs to Send Request
stage: explainer
date: 2025-12-09
surface: agent
model: gemini
feature: rag-chatbot-integration
branch: main
user: unknown
command: how do i send a request from ther
labels: [fastapi, debugging, explainer]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - none
tests:
 - none
---

## Prompt

how do i send a request from ther

## Response snapshot

Great! So you can see the documentation page at `http://localhost:8000/docs`. This is good news, it means your backend server is running.

Now, to send a request from that page:
1. Look for the section for the `/api/rag-chat` endpoint. It should have a green `POST` button next to it.
2. Click on that section to expand it. You will see details about the endpoint, including the request body.
3. On the right side of the expanded section, you will see a 'Try it out' button. Click it.
4. The 'Request body' section will become an editable text area. It will have some example JSON in it. It should look something like this:
```json
{
  "query": "string",
  "selected_text": "string"
}
```
5. You can change the value of `"query"` to any question you want to ask. For example: `"query": "What is ROS?"`. You can leave `"selected_text"` as it is, or remove that line.
6. Once you have edited the query, click the blue 'Execute' button.
7. The server will process the request, and you will see the response in a 'Server response' section that appears below. It will show the response body, response headers, and the `curl` command for the request.

Please let me know what you see in the 'Server response' section. If there is an error, the response body will contain the error message from the backend.

## Outcome

- ✅ Impact: Provided clear instructions to the user on how to use the FastAPI documentation to test the API endpoint directly.
- 🧪 Tests: No new tests were added.
- 📁 Files: No files were modified.
- 🔁 Next prompts: User to report the response from the API endpoint.
- 🧠 Reflection: When guiding a user through debugging, it's important to provide very clear, step-by-step instructions, especially when they are not familiar with the tools.

## Evaluation notes (flywheel)

- Failure modes observed: The user might still have trouble finding the elements on the page.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): none
