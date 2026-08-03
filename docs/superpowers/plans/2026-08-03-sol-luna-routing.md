# Sol-Luna Task Routing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a global Codex instruction that asks for permission to delegate eligible Sol work to `gpt-5.6-luna`, then keeps final responsibility with Sol.

**Architecture:** Persist the policy in the user's global `AGENTS.md`, because this is a cross-repository personal behavior rather than a firmware-project setting. The rule performs a lightweight eligibility check, asks once before delegation, constrains Luna execution, and falls back to Sol without changing the configured provider or default model.

**Tech Stack:** Codex global Markdown guidance, PowerShell verification, RTK command proxy

## Global Constraints

- Modify only `C:\Users\zhuzhichao\.codex\AGENTS.md` during implementation.
- Preserve every existing personal-default rule and the existing `RTK.md` include.
- Trigger only when the current main model is `gpt-5.6-sol`.
- Ask only when the task has at least two independent subtasks and Luna delegation is expected to reduce Sol consumption.
- Require explicit user approval before creating any Luna task.
- Use exactly `gpt-5.6-luna`; never silently substitute another model.
- Create at most two Luna tasks and prohibit nested delegation.
- Keep architecture, risk decisions, conflict handling, integration, and final verification with Sol.
- Do not modify firmware source, Codex provider, authentication, quota settings, or the configured default model.

---

### Task 1: Add and verify the global Sol-Luna routing policy

**Files:**
- Modify: `C:\Users\zhuzhichao\.codex\AGENTS.md`
- Reference: `docs/superpowers/specs/2026-08-03-sol-luna-routing-design.md`
- Test: Inline PowerShell assertions against `C:\Users\zhuzhichao\.codex\AGENTS.md`

**Interfaces:**
- Consumes: Codex's global `AGENTS.md` personal-guidance loading behavior.
- Produces: A `## Sol/Luna Cost-Saving Delegation` guidance section that controls when Sol asks for permission and how approved Luna tasks are constrained.

- [ ] **Step 1: Verify the routing policy is not already installed**

Run:

```powershell
rtk proxy powershell.exe -NoProfile -Command '$p="C:\Users\zhuzhichao\.codex\AGENTS.md"; $t=Get-Content -LiteralPath $p -Raw; if ($t -notmatch [regex]::Escape("## Sol/Luna Cost-Saving Delegation")) { throw "EXPECTED FAILURE: Sol/Luna routing policy is not installed" }'
```

Expected: command fails with `EXPECTED FAILURE: Sol/Luna routing policy is not installed`.

- [ ] **Step 2: Append the minimal global guidance**

Use `apply_patch` to append this exact section after the existing `RTK.md` include:

```markdown

## Sol/Luna Cost-Saving Delegation

- When the current main model is `gpt-5.6-sol`, first perform a lightweight task decomposition.
- Ask about delegation only when the task contains at least two independent subtasks and using a lower-cost model is expected to reduce Sol consumption after coordination overhead.
- Use this one-time question: “该任务可拆成 N 个独立子任务。是否使用 gpt-5.6-luna 执行子任务，由 Sol 负责最终复核？”
- Do not create delegated tasks unless the user explicitly agrees. If the user declines, continue with Sol alone without asking again for the same task.
- After approval, use exactly `gpt-5.6-luna` for at most two concurrent tasks. Do not silently substitute another model if Luna is unavailable; tell the user instead.
- Do not allow delegated Luna tasks to create nested agents or tasks.
- Give every Luna task a bounded scope, inputs, forbidden actions, verification command, and concise return format.
- Do not let concurrent writing tasks modify the same file or shared mutable state.
- Delegate search, research, mechanical edits, independent tests, builds, and log analysis. Keep architecture, interfaces, security, risk decisions, conflict handling, integration, and final verification with Sol.
- Do not ask for delegation for ordinary questions, single-file small edits, strongly sequential work, or tasks where duplicated context and coordination are unlikely to save Sol usage.
```

Expected: the existing personal defaults and `@C:\Users\zhuzhichao\.codex\RTK.md` remain unchanged above the new section.

- [ ] **Step 3: Run structural and policy assertions**

Run:

```powershell
rtk proxy powershell.exe -NoProfile -Command '$p="C:\Users\zhuzhichao\.codex\AGENTS.md"; $t=Get-Content -LiteralPath $p -Raw; $required=@("Default to Simplified Chinese","@C:\Users\zhuzhichao\.codex\RTK.md","## Sol/Luna Cost-Saving Delegation","gpt-5.6-sol","at least two independent subtasks","explicitly agrees","gpt-5.6-luna","at most two concurrent tasks","nested agents or tasks","final verification with Sol"); $missing=@($required | Where-Object { $t -notmatch [regex]::Escape($_) }); if ($missing.Count) { throw ("Missing required guidance: " + ($missing -join ", ")) }; "PASS: global Sol/Luna routing policy is complete"'
```

Expected: `PASS: global Sol/Luna routing policy is complete`.

- [ ] **Step 4: Confirm the change is isolated**

Run:

```powershell
rtk git status --short
```

Expected: no new firmware-source changes from this task. The pre-existing Keil build-output and user-settings modifications may remain listed.

- [ ] **Step 5: Report activation behavior**

Report that the global guidance is installed and applies to newly evaluated Sol tasks. Advise starting a new task if an already-running task does not reload global guidance. Do not claim that the policy changes billing multipliers or guarantees savings; it only controls delegation behavior.

No Git commit is created for `C:\Users\zhuzhichao\.codex\AGENTS.md` because it is a personal global file outside this repository.
