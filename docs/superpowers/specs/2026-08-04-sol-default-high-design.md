# Sol 默认 High 思考强度设计

## 目标

将 Codex 全局主模型 `gpt-5.6-sol` 的默认思考强度从 `xhigh` 降为
`high`，减少主控任务的推理消耗，同时保留现有 Sol/Luna 自适应委派策略。

## 变更

只修改全局配置文件：

`C:\Users\zhuzhichao\.codex\config.toml`

将：

```toml
model_reasoning_effort = "xhigh"
```

替换为：

```toml
model_reasoning_effort = "high"
```

## 保持不变

- 主模型仍为 `gpt-5.6-sol`。
- 当前模型供应商及模型目录配置保持不变。
- 全局 `AGENTS.md` 和 Luna 的 `high→max` 自适应策略保持不变。
- 固件源码、Keil 工程设置和既有工作区改动不受影响。

## 验证

修改后验证：

1. `config.toml` 中 `model = "gpt-5.6-sol"` 仍只出现一次。
2. `model_reasoning_effort = "high"` 只出现一次。
3. `model_reasoning_effort = "xhigh"` 不再作为活动配置出现。
4. `AGENTS.md` 仍包含 Luna 默认 `high`、复杂或首轮失败升级 `max` 的规则。
5. 固件仓库状态与修改前基线一致。

该全局设置在新建 Codex 任务后可靠加载；不对当前已运行任务承诺热重载。
