# Sol-Luna Adaptive Reasoning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make delegated Luna tasks use `high` reasoning by default and escalate once to `max` only for complex work or a failed first attempt.

**Architecture:** Update the existing global Sol/Luna routing section in the user's `AGENTS.md`; do not add a second policy section. The policy selects the initial Luna reasoning level, defines one bounded escalation path, preserves Sol as final decision-maker, and leaves the configured Sol model and effort unchanged.

**Tech Stack:** Codex global Markdown guidance, PowerShell verification, RTK command proxy

## Global Constraints

- Modify only `C:\Users\zhuzhichao\.codex\AGENTS.md` during implementation.
- Preserve all existing personal-default and Sol/Luna delegation rules unless this plan explicitly replaces their wording.
- Use exactly `gpt-5.6-luna` with reasoning effort `high` by default.
- Use Luna `max` directly only for a delegated complex subtask that meets an explicit complexity criterion.
- Allow at most one `high` to `max` escalation per Luna subtask.
- Stop Luna retries and return evidence to Sol after a failed `max` attempt.
- Keep architecture, security, risk decisions, conflict handling, integration, and final verification with Sol.
- Preserve `gpt-5.6-sol` and its current `xhigh` reasoning configuration.
- Do not modify firmware source, Codex provider, authentication, quota settings, or unrelated workspace changes.

---

### Task 1: Add adaptive Luna reasoning to the global routing policy

**Files:**
- Modify: `C:\Users\zhuzhichao\.codex\AGENTS.md`
- Reference: `docs/superpowers/specs/2026-08-04-sol-luna-adaptive-reasoning-design.md`
- Test: Inline PowerShell assertions against `C:\Users\zhuzhichao\.codex\AGENTS.md`

**Interfaces:**
- Consumes: The existing `## Sol/Luna Cost-Saving Delegation` section and Codex task creation with Luna `high` or `max`.
- Produces: One global routing section with an explicit default effort, complexity gate, single escalation, stopping rule, and updated authorization question.

- [ ] **Step 1: Run a failing assertion for the adaptive policy**

Run:

```powershell
rtk proxy powershell.exe -NoProfile -Command '$p="C:\Users\zhuzhichao\.codex\AGENTS.md"; $t=Get-Content -LiteralPath $p -Raw -Encoding UTF8; if ($t -notmatch [regex]::Escape("默认 high，复杂任务或首轮失败升级 max")) { throw "EXPECTED FAILURE: adaptive Luna reasoning policy is not installed" }'
```

Expected: command fails with `EXPECTED FAILURE: adaptive Luna reasoning policy is not installed`.

- [ ] **Step 2: Replace the authorization question and add the effort rules**

Use `apply_patch` to make these exact changes inside the existing
`## Sol/Luna Cost-Saving Delegation` section:

```diff
-- Use this one-time question: “该任务可拆成 N 个独立子任务。是否使用 gpt-5.6-luna 执行子任务，由 Sol 负责最终复核？”
+- Use this one-time question: “该任务可拆成 N 个独立子任务。是否使用 gpt-5.6-luna 执行子任务（默认 high，复杂任务或首轮失败升级 max），由 Sol 负责最终复核？”
 - Do not create delegated tasks unless the user explicitly agrees. If the user declines, continue with Sol alone without asking again for the same task.
-- After approval, use exactly `gpt-5.6-luna` for at most two concurrent tasks. Do not silently substitute another model if Luna is unavailable; tell the user instead.
+- After approval, use exactly `gpt-5.6-luna` at reasoning effort `high` by default for at most two concurrent tasks. Do not silently substitute another model or effort if the requested Luna configuration is unavailable; tell the user instead.
+- Start a delegated Luna task at reasoning effort `max` only when it requires cross-module interface analysis, concurrency, interrupts, timing, shared-state reasoning, security or data-integrity analysis, ambiguous root-cause debugging, or large-context synthesis. Sol still owns final architecture, security, and risk decisions.
+- If a Luna `high` attempt fails its required verification, omits a required deliverable or evidence, or is rejected by Sol for a requirement gap, conflict, or scope violation, continue the same task at reasoning effort `max` at most once.
+- If the Luna `max` attempt fails, stop Luna retries and return the failure evidence to Sol.
```

Expected: the section heading remains unique, the new rules appear directly after the approval rule, and all unrelated guidance remains byte-for-byte unchanged.

- [ ] **Step 3: Run structural and behavioral assertions**

Run:

```powershell
rtk proxy powershell.exe -NoProfile -Command '$p="C:\Users\zhuzhichao\.codex\AGENTS.md"; $t=Get-Content -LiteralPath $p -Raw -Encoding UTF8; $required=@("## Sol/Luna Cost-Saving Delegation","默认 high，复杂任务或首轮失败升级 max","reasoning effort ``high`` by default","reasoning effort ``max`` only when","continue the same task at reasoning effort ``max`` at most once","If the Luna ``max`` attempt fails, stop Luna retries","Sol still owns final architecture","at most two concurrent tasks","Do not allow delegated Luna tasks to create nested agents or tasks"); $missing=@($required | Where-Object { -not $t.Contains($_) }); $headingCount=([regex]::Matches($t,[regex]::Escape("## Sol/Luna Cost-Saving Delegation"))).Count; if ($missing.Count) { throw ("Missing required guidance: " + ($missing -join ", ")) }; if ($headingCount -ne 1) { throw "Sol/Luna section count is $headingCount" }; "PASS: adaptive Luna reasoning policy is complete and unique"'
```

Expected: `PASS: adaptive Luna reasoning policy is complete and unique`.

- [ ] **Step 4: Verify the configured Sol model and repository isolation**

Run:

```powershell
rtk rg -n "^model\s*=|^model_reasoning_effort\s*=" "C:\Users\zhuzhichao\.codex\config.toml"
```

Expected:

```text
model = "gpt-5.6-sol"
model_reasoning_effort = "xhigh"
```

Run:

```powershell
rtk git status --short
```

Expected: no new firmware-source or project-setting changes from this implementation. The pre-existing Keil output and user-setting modifications may remain listed.

- [ ] **Step 5: Report activation and remaining optimization**

Report that new Sol tasks will request Luna `high` by default, select Luna `max` for explicitly complex delegated work, and perform at most one failed-attempt escalation. Recommend evaluating `Sol/high + Luna/high→max` later with representative tasks as the largest remaining cost optimization, without changing Sol in this implementation.

No Git commit is created for `C:\Users\zhuzhichao\.codex\AGENTS.md` because it is a personal global file outside this repository.
