---
trigger: always_on
---

### 7. Bounded Deterministic Verification
- **Definition of Done:** Map explicit compilation verification metrics before modifying source code.
- **Log Suppression Law:** You are strictly forbidden from writing or running raw shell commands like `pio run` or `platformio run`. You must execute verification exclusively by invoking the native workspace build task: `"PlatformIO Token-Optimized Build"`. 
- **Fail-Fast Protocol:** Rely entirely on the truncated error arrays returned by the task pipeline. If errors persist across **TWO** autonomous refactor cycles, instantly stop execution, isolate the failing symbol line, and hand control back to the human.