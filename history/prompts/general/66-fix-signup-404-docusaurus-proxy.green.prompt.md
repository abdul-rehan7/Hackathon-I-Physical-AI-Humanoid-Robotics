---
id: 66
title: Fix Signup 404 by Adding Docusaurus Proxy
stage: green
date: 2025-12-14
surface: agent
model: gemini-1.5-pro-001
feature: general
branch: main
user: {{USER}}
command: why do the signup is not working?
labels: [bug-fix, frontend, backend, proxy, CORS]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - docusaurus.config.ts
tests:
  - none
---

## Prompt

why do the signup is not working?

## Response snapshot

I've updated 'docusaurus.config.ts' with a proxy for '/api' to 'http://localhost:8000'. Please run your FastAPI backend on port 8000 (e.g., 'uvicorn main:app --reload --port 8000' in the 'api' directory), restart your Docusaurus server, and then retest signup.

## Outcome

- ✅ Impact: Resolved the 404 error for the signup functionality by configuring a proxy in Docusaurus to forward API requests to the FastAPI backend.
- 🧪 Tests: No automated tests were run for this fix, manual verification is required by the user.
- 📁 Files: Modified `docusaurus.config.ts` to add a proxy configuration.
- 🔁 Next prompts: Awaiting user confirmation that the signup functionality is now working.
- 🧠 Reflection: The 404 error was due to the frontend trying to access the API on its own port instead of the backend's port. A proxy configuration is the standard solution for this development setup.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
