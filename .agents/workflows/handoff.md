---
description: Serializes workspace context to a file to prepare for a clean session reset
---

You are executing a context serialization protocol. Perform these steps sequentially without conversational filler:

1. Parse the current working memory tree, active git status, and compilation state.
2. Write or update a structured file at `docs/HANDOFF.md`. Do not add any conversational markdown introduction or conclusion text to the chat window.

### Core Content Schema for docs/HANDOFF.md:
# Handoff State Matrix
- **Target Subsystem:** [Identify the active module, e.g., Airbrake Control / MEKF Filter]
- **Modified Assets:** [Comma-separated relative file paths modified during this active session]
- **Last Verification Status:** [SUCCESS or FAILED with isolated compilation error line from build.log]
- **Immediate Task Vector:** [Single precise sentence outlining the next logical line of code or calculation to execute]

3. Once the file is verified on disk, output exactly this literal string to the chat window: `[STATE_SERIALIZED: CLEAR_CONTEXT_NOW]`