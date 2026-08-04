# Sol Default High Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Change the global `gpt-5.6-sol` reasoning effort from `xhigh` to `high` without changing the model, provider, Luna policy, or firmware repository.

**Architecture:** Perform one exact replacement in the user's global `config.toml`. Validate the active model and effort structurally, verify the Luna adaptive policy remains present, and compare the firmware repository status with its pre-change baseline.

**Tech Stack:** Codex TOML configuration, PowerShell verification, RTK command proxy

## Global Constraints

- Modify only `C:\Users\zhuzhichao\.codex\config.toml` during implementation.
- Replace only `model_reasoning_effort = "xhigh"` with `model_reasoning_effort = "high"`.
- Preserve `model = "gpt-5.6-sol"`.
- Preserve the current model provider and model catalog configuration.
- Preserve the global `AGENTS.md` and Luna `high→max` adaptive policy.
- Do not modify firmware source, Keil project settings, or unrelated workspace changes.

---

### Task 1: Set the global Sol reasoning effort to high

**Files:**
- Modify: `C:\Users\zhuzhichao\.codex\config.toml`
- Reference: `docs/superpowers/specs/2026-08-04-sol-default-high-design.md`
- Test: Inline PowerShell assertions against global Codex configuration

**Interfaces:**
- Consumes: The existing global `model = "gpt-5.6-sol"` and `model_reasoning_effort = "xhigh"` settings.
- Produces: A single active `model_reasoning_effort = "high"` setting for new Codex tasks.

- [ ] **Step 1: Run a failing assertion for the requested effort**

Run:

```powershell
rtk proxy powershell.exe -NoProfile -Command '$p="C:\Users\zhuzhichao\.codex\config.toml"; $t=Get-Content -LiteralPath $p -Raw -Encoding UTF8; if (-not $t.Contains("model_reasoning_effort = ""high""")) { throw "EXPECTED FAILURE: global Sol reasoning effort is not high" }'
```

Expected: command fails with
`EXPECTED FAILURE: global Sol reasoning effort is not high`.

- [ ] **Step 2: Replace the one active effort setting**

Use `apply_patch` for this exact replacement:

```diff
-model_reasoning_effort = "xhigh"
+model_reasoning_effort = "high"
```

Expected: no other line in `config.toml` changes.

- [ ] **Step 3: Verify the active model and effort**

Run:

```powershell
rtk proxy powershell.exe -NoProfile -Command '$p="C:\Users\zhuzhichao\.codex\config.toml"; $lines=Get-Content -LiteralPath $p -Encoding UTF8; $sol=@($lines | Where-Object { $_ -eq "model = ""gpt-5.6-sol""" }); $high=@($lines | Where-Object { $_ -eq "model_reasoning_effort = ""high""" }); $xhigh=@($lines | Where-Object { $_ -eq "model_reasoning_effort = ""xhigh""" }); if ($sol.Count -ne 1) { throw "Active Sol model count is $($sol.Count)" }; if ($high.Count -ne 1) { throw "Active high effort count is $($high.Count)" }; if ($xhigh.Count -ne 0) { throw "Active xhigh effort count is $($xhigh.Count)" }; "PASS: global Sol model is unchanged and reasoning effort is high"'
```

Expected:
`PASS: global Sol model is unchanged and reasoning effort is high`.

- [ ] **Step 4: Verify preserved global guidance and repository isolation**

Run:

```powershell
rtk proxy powershell.exe -NoProfile -Command '$p="C:\Users\zhuzhichao\.codex\AGENTS.md"; $t=Get-Content -LiteralPath $p -Raw -Encoding UTF8; $required=@("gpt-5.6-luna","默认 high，复杂任务或首轮失败升级 max","at reasoning effort ``high`` by default","at reasoning effort ``max`` at most once"); $missing=@($required | Where-Object { -not $t.Contains($_) }); if ($missing.Count) { throw ("Missing Luna guidance: " + ($missing -join ", ")) }; "PASS: Luna adaptive policy is preserved"'
```

Expected: `PASS: Luna adaptive policy is preserved`.

Run:

```powershell
rtk git status --short
```

Expected: no new firmware or project-setting changes from this task. The pre-existing Keil output and user-setting modifications may remain listed.

- [ ] **Step 5: Report activation**

Report that new Codex tasks will use `gpt-5.6-sol` at `high` by default and that Luna remains `high→max`. Recommend creating a new task so the global configuration is reloaded.

No Git commit is created for `C:\Users\zhuzhichao\.codex\config.toml` because it is a personal global file outside this repository.
