---
id: 7
title: Update Constitution with New Principles
stage: constitution
date: 2025-12-06
surface: "agent"
model: "gemini-pro"
feature: "none"
branch: "none"
user: "user"
command: "Edit the file '.specify/memory/constitution.md' to strictly enforce the following rules:\n\n1.  **Technical Platform Principle:** All Docusaurus components, configurations, and React code must use the **TypeScript** language.\n2.  **Context7 Principle:** All code generation and configuration changes must be **explicitly prompted to use Context7** for version-accurate Docusaurus, ROS 2, and NVIDIA Isaac SDK documentation.\n3.  **Content Principle:** The book's content must be organized into **weekly modules** that mirror the structure of the provided course breakdown (Weeks 1-13) and use **Markdown heading levels (H2, H3)** to create the Table of Contents.\n4.  **Hardware Principle:** A dedicated section must be created in the docs for **\"Hardware Requirements\"** detailing the Workstation and Edge Kit specifications."
labels: ["constitution", "docusaurus", "rules"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

Edit the file '.specify/memory/constitution.md' to strictly enforce the following rules:

1.  **Technical Platform Principle:** All Docusaurus components, configurations, and React code must use the **TypeScript** language.
2.  **Context7 Principle:** All code generation and configuration changes must be **explicitly prompted to use Context7** for version-accurate Docusaurus, ROS 2, and NVIDIA Isaac SDK documentation.
3.  **Content Principle:** The book's content must be organized into **weekly modules** that mirror the structure of the provided course breakdown (Weeks 1-13) and use **Markdown heading levels (H2, H3)** to create the Table of Contents.
4.  **Hardware Principle:** A dedicated section must be created in the docs for **"Hardware Requirements"** detailing the Workstation and Edge Kit specifications.

## Response snapshot

I have updated the `.specify/memory/constitution.md` file to include the new principles.

## Outcome

- ✅ Impact: The project's constitution has been updated with four new principles regarding technical platform, context, content, and hardware.
- 🧪 Tests: No tests were run.
- 📁 Files: Modified 1 file: `.specify/memory/constitution.md`.
- 🔁 Next prompts: The user might want to create or modify documentation based on the new principles.
- 🧠 Reflection: The process of updating the constitution file was straightforward.

## Evaluation notes (flywheel)

- Failure modes observed: "none"
- Graders run and results (PASS/FAIL): "PASS"
- Prompt variant (if applicable): "none"
- Next experiment (smallest change to try): "none"
